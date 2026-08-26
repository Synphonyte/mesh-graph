#!/usr/bin/env python3
"""Applies the mesh-graph "cascade check" instrumentation to a copy of the repo.

Usage: python3 apply_instrumentation.py /path/to/mesh-graph-copy

Idempotent: safe to run repeatedly (asserts each anchor is present exactly once,
so a second run on an already-patched tree fails loudly instead of double-patching).

What it adds (debug fork only, never commit):
- src/ops/cascade_check.rs: per-op mesh stats (element counts, edge-length
  distribution, degenerate-face counts) + a topology-validity scan
  (missing/asymmetric twins, stale next/outgoing/face pointers).
- subdivide_until_edges_below_max_length / collapse_until_edges_above_min_length:
  entry/exit + every-500th-iteration stats; first-20-iteration detail dump for ops
  with >100 pending edges (subdivided edge, adjacent face vertices, created
  halfedges).
- merge_vertices_one_rings: entry/exit stats + one-ring dump + topo scan.
- rebuild_outgoing_halfedges: normalizes stale per-vertex `outgoing_halfedge`
  seed pointers (keeps the old seed when live, else picks a live member).
- Cargo.toml: drops `release_max_level_off` from tracing, adds a dummy
  `bookkeeping-check` feature.
"""
import re
import sys

def patch(path, edits, tag):
    with open(path) as f:
        src = f.read()
    for old, new, count in edits:
        n = src.count(old)
        if n != count:
            raise SystemExit(f"[{tag}] anchor found {n}x (expected {count}): {old[:80]!r}")
        src = src.replace(old, new)
    with open(path, "w") as f:
        f.write(src)
    print(f"patched {path}")

CASCADE_CHECK_RS = r'''//! Mesh growth/degeneration counters ("cascade check") for the item-3 investigation.
//!
//! Logs per-operation mesh statistics: element counts, edge-length distribution
//! (long / short / zero / NaN), and degenerate (zero-area) face counts. This
//! distinguishes "the mesh is genuinely huge" (entry state already blown up) from
//! "the op creates work faster than it consumes" (counts grow inside the op).
//!
//! Debug fork instrumentation: unconditional compile, events only fire when the
//! `mesh_graph::ops::cascade_check` target is enabled via RUST_LOG.

use tracing::{Level, enabled, info};

use crate::MeshGraph;

#[derive(Debug, Default)]
pub(super) struct MeshStats {
    pub vertices: usize,
    pub halfedges: usize,
    pub edges: usize,
    pub faces: usize,
    pub long_edges: usize,
    pub short_edges: usize,
    pub zero_edges: usize,
    pub nan_edges: usize,
    pub invalid_edges: usize,
    pub degen_faces: usize,
    pub invalid_faces: usize,
    pub min_len_sqr: f32,
    pub max_len_sqr: f32,
}

pub(super) fn collect(
    mg: &MeshGraph,
    max_len_sqr: Option<f32>,
    min_len_sqr: Option<f32>,
) -> MeshStats {
    let mut stats = MeshStats {
        vertices: mg.vertices.len(),
        halfedges: mg.halfedges.len(),
        faces: mg.faces.len(),
        min_len_sqr: f32::INFINITY,
        max_len_sqr: 0.0,
        ..Default::default()
    };

    // Count each undirected edge once (halfedge pair), like `halfedges_map`.
    for (he_id, he) in &mg.halfedges {
        let Some(twin_id) = he.twin else {
            stats.invalid_edges += 1;
            continue;
        };

        if he_id > twin_id {
            continue;
        }

        stats.edges += 1;

        let Some(start_v) = he.start_vertex(mg) else {
            stats.invalid_edges += 1;
            continue;
        };

        let end_v = he.end_vertex;

        let depth = match (mg.positions.get(start_v), mg.positions.get(end_v)) {
            (Some(&start), Some(&end)) => (start - end).length_squared(),
            _ => {
                stats.invalid_edges += 1;
                continue;
            }
        };

        stats.min_len_sqr = stats.min_len_sqr.min(depth);
        stats.max_len_sqr = stats.max_len_sqr.max(depth);

        if depth.is_nan() {
            stats.nan_edges += 1;
        } else if depth == 0.0 {
            stats.zero_edges += 1;
        }

        if max_len_sqr.is_some_and(|t| depth > t) {
            stats.long_edges += 1;
        }
        if min_len_sqr.is_some_and(|t| depth < t) {
            stats.short_edges += 1;
        }
    }

    // Degenerate (zero-area) faces: identical vertices or coincident positions.
    for face in mg.faces.values() {
        let positions: Vec<_> = face
            .vertices(mg)
            .filter_map(|v| mg.positions.get(v).copied())
            .collect();

        if positions.len() < 3 {
            stats.invalid_faces += 1;
            continue;
        }

        let a = positions[1] - positions[0];
        let b = positions[2] - positions[0];
        let area_sqr = a.cross(b).length_squared();

        if area_sqr == 0.0 || area_sqr.is_nan() {
            stats.degen_faces += 1;
        }
    }

    stats
}

pub(super) fn log_op(
    mg: &MeshGraph,
    label: &str,
    max_len_sqr: Option<f32>,
    min_len_sqr: Option<f32>,
    extra: &str,
) {
    if !enabled!(Level::INFO) {
        // guard the (cheap but non-trivial) collection when nothing is subscribed
        return;
    }

    let s = collect(mg, max_len_sqr, min_len_sqr);

    info!(
        "cascade: {label}: verts={} hes={} edges={} faces={} long={} short={} zero={} nan={} invalid_edges={} degen_faces={} invalid_faces={} len_sqr=[{:.3e}..{:.3e}] {extra}",
        s.vertices,
        s.halfedges,
        s.edges,
        s.faces,
        s.long_edges,
        s.short_edges,
        s.zero_edges,
        s.nan_edges,
        s.invalid_edges,
        s.degen_faces,
        s.invalid_faces,
        s.min_len_sqr,
        s.max_len_sqr,
    );
}

// -- topology validity scan ---------------------------------------------------

/// Counts broken topology invariants that the per-vertex list check cannot see:
/// missing/asymmetric twins, halfedges referencing dead vertices, stale halfedge
/// `next` pointers, stale vertex `outgoing_halfedge` pointers and stale face
/// halfedge pointers.
#[derive(Debug, Default)]
pub(super) struct TopoValidity {
    pub halfedges: usize,
    pub missing_twin: usize,
    pub asym_twin: usize,
    pub dead_end_vertex: usize,
    pub stale_next: usize,
    pub stale_vertex_outgoing: usize,
    pub stale_face_he: usize,
    /// First few offending halfedge ids, to identify the corruption site.
    pub sample_ids: Vec<crate::HalfedgeId>,
}

/// Scans the halfedge topology for broken links. O(H + V + F).
pub(super) fn topo_validity(mg: &MeshGraph) -> TopoValidity {
    let mut v = TopoValidity::default();

    let mut note = |he_id: crate::HalfedgeId, v: &mut TopoValidity| {
        if v.sample_ids.len() < 8 {
            v.sample_ids.push(he_id);
        }
    };

    for (he_id, he) in &mg.halfedges {
        v.halfedges += 1;

        let Some(twin_id) = he.twin else {
            v.missing_twin += 1;
            note(he_id, &mut v);
            continue;
        };

        match mg.halfedges.get(twin_id) {
            Some(twin) => {
                if twin.twin != Some(he_id) {
                    v.asym_twin += 1;
                    note(he_id, &mut v);
                }
            }
            None => {
                v.asym_twin += 1;
                note(he_id, &mut v);
            }
        }

        if !mg.vertices.contains_key(he.end_vertex) {
            v.dead_end_vertex += 1;
            note(he_id, &mut v);
        }

        if let Some(next_id) = he.next {
            if !mg.halfedges.contains_key(next_id) {
                v.stale_next += 1;
                note(he_id, &mut v);
            }
        }
    }

    for (v_id, vertex) in &mg.vertices {
        if let Some(oh) = vertex.outgoing_halfedge {
            if !mg.halfedges.contains_key(oh) {
                v.stale_vertex_outgoing += 1;
                note(oh, &mut v);
            }
        }
    }

    for (face_id, face) in &mg.faces {
        if !mg.halfedges.contains_key(face.halfedge) {
            v.stale_face_he += 1;
            note(face.halfedge, &mut v);
        }
    }

    v
}

pub(super) fn log_topo(mg: &MeshGraph, label: &str) {
    if !enabled!(Level::INFO) {
        return;
    }

    let v = topo_validity(mg);

    if v.missing_twin > 0
        || v.asym_twin > 0
        || v.dead_end_vertex > 0
        || v.stale_next > 0
        || v.stale_vertex_outgoing > 0
        || v.stale_face_he > 0
    {
        info!(
            "topo {label}: BAD missing_twin={} asym_twin={} dead_end={} stale_next={} stale_v_out={} stale_face_he={} sample={:?}",
            v.missing_twin,
            v.asym_twin,
            v.dead_end_vertex,
            v.stale_next,
            v.stale_vertex_outgoing,
            v.stale_face_he,
            v.sample_ids,
        );
    }
}
'''

def main(root: str):
    ops = f"{root}/src/ops"

    # -- Cargo.toml ----------------------------------------------------------
    toml = f"{root}/Cargo.toml"
    with open(toml) as f:
        t = f.read()
    if 'features = ["release_max_level_off"]' in t:
        t = t.replace('features = ["release_max_level_off"]', "features = []")
    if "bookkeeping-check" not in t:
        m = re.search(r"^\[features\]\n(.*?)(?=\n\[|\Z)", t, re.S | re.M)
        assert m, "[features] section not found"
        t = t[: m.end()] + "\nbookkeeping-check = []" + t[m.end() :]
    with open(toml, "w") as f:
        f.write(t)
    print(f"patched {toml}")

    # -- cascade_check.rs ----------------------------------------------------
    with open(f"{ops}/cascade_check.rs", "w") as f:
        f.write(CASCADE_CHECK_RS)
    print(f"wrote {ops}/cascade_check.rs")

    # -- ops/mod.rs ----------------------------------------------------------
    patch(f"{ops}/mod.rs", [
        ("mod transform;\n", "mod cascade_check;\nmod transform;\n", 1),
    ], "mod.rs")

    # -- subdivide.rs --------------------------------------------------------
    patch(f"{ops}/subdivide.rs", [
        # entry stats + topo
        (
            "        let mut halfedges_to_subdivide = self.halfedges_map(|len_sqr| len_sqr > max_length_squared);\n",
            "        let mut halfedges_to_subdivide = self.halfedges_map(|len_sqr| len_sqr > max_length_squared);\n\n        super::cascade_check::log_op(self, \"subdivide entry\", Some(max_length_squared), None, &format!(\"set={}\", halfedges_to_subdivide.len()));\n        super::cascade_check::log_topo(self, \"subdivide entry\");\n\n        // Bounded work: stretched regions can re-feed the set forever (fan edges above\n        // threshold), and the old `halfedges.len()` bound let the op blow up the mesh.\n        let budget = halfedges_to_subdivide.len() * 2 + 100;\n",
            1,
        ),
        # iter counter
        (
            "        for _ in 0..self.halfedges.len() {\n            if halfedges_to_subdivide.is_empty() {",
            "        let mut cascade_iter = 0usize;\n\n        for _ in 0..budget {\n            if halfedges_to_subdivide.is_empty() {",
            1,
        ),
        # detail dump after max-picking
        (
            "            for (&he, &len) in &halfedges_to_subdivide {\n                if len > max_len {\n                    max_len = len;\n                    max_he_id = he;\n                }\n            }\n\n            halfedges_to_subdivide.remove(&max_he_id);\n",
            "            for (&he, &len) in &halfedges_to_subdivide {\n                if len > max_len {\n                    max_len = len;\n                    max_he_id = he;\n                }\n            }\n\n            halfedges_to_subdivide.remove(&max_he_id);\n\n            if cascade_iter < 20 && halfedges_to_subdivide.len() > 100 {\n                tracing::info!(target: \"mesh_graph::ops::cascade_check\", \"detail iter {}: subdividing he={:?} len_sqr={} set_len={} (gate set>100)\", cascade_iter, max_he_id, max_len, halfedges_to_subdivide.len() + 1);\n                let det_he = self.halfedges.get(max_he_id);\n                if let Some(det_he) = det_he {\n                    let det_start = det_he.start_vertex(self);\n                    let det_end = det_he.end_vertex;\n                    let det_pos = |v: Option<crate::VertexId>| {\n                        v.and_then(|v| self.positions.get(v).copied())\n                            .map(|p| format!(\"{p:?}\"))\n                            .unwrap_or_else(|| \"?\".to_string())\n                    };\n                    tracing::info!(target: \"mesh_graph::ops::cascade_check\", \"detail iter {}: start_v={:?} pos={} end_v={:?} pos={} twin={:?} face={:?}\", cascade_iter, det_start, det_pos(det_start), det_end, det_pos(Some(det_end)), det_he.twin, det_he.face);\n                    if let Some(twin) = det_he.twin {\n                        if let Some(twin_he) = self.halfedges.get(twin) {\n                            tracing::info!(target: \"mesh_graph::ops::cascade_check\", \"detail iter {}: twin face={:?}\", cascade_iter, twin_he.face);\n                        }\n                    }\n\n                    for (flabel, fid) in [(\"face\", det_he.face), (\"twin_face\", if let Some(t) = det_he.twin { self.halfedges.get(t).and_then(|h| h.face) } else { None })] {\n                        if let Some(fid) = fid {\n                            if let Some(f) = self.faces.get(fid) {\n                                let verts: Vec<String> = f\n                                    .vertices(self)\n                                    .map(|v| {\n                                        let pos = self.positions.get(v).copied().map(|p| format!(\"{p:?}\")).unwrap_or_else(|| \"?\".to_string());\n                                        format!(\"{v:?}@{pos}\")\n                                    })\n                                    .collect();\n                                tracing::info!(target: \"mesh_graph::ops::cascade_check\", \"detail iter {}: {flabel} {fid:?} verts=[{}]\", cascade_iter, verts.join(\", \"));\n                            }\n                        }\n                    }\n                }\n            }\n",
            1,
        ),
        # created-halfedge dump after subdivide_edge
        (
            "            let subdivide_edge_result =\n                unwrap_or_return!(self.subdivide_edge(max_he_id), \"Couldn't subdivide edge\");\n",
            "            let subdivide_edge_result =\n                unwrap_or_return!(self.subdivide_edge(max_he_id), \"Couldn't subdivide edge\");\n\n            if cascade_iter < 20 && halfedges_to_subdivide.len() > 100 {\n                for &created_he_id in &subdivide_edge_result.added_halfedges {\n                    if let Some(nhe) = self.halfedges.get(created_he_id) {\n                        let sv = nhe.start_vertex(self);\n                        let ev = nhe.end_vertex;\n                        let poss = |v: Option<crate::VertexId>| {\n                            v.and_then(|v| self.positions.get(v).copied())\n                                .map(|p| format!(\"{p:?}\"))\n                                .unwrap_or_else(|| \"?\".to_string())\n                        };\n                        tracing::info!(target: \"mesh_graph::ops::cascade_check\", \"detail iter {}: created he={:?} start={:?}@{} end={:?}@{} face={:?} twin={:?}\", cascade_iter, created_he_id, sv, poss(sv), ev, poss(Some(ev)), nhe.face, nhe.twin);\n                    }\n                }\n            }\n",
            1,
        ),
        # per-500 + exit stats + topo
        (
            "                    halfedges_to_subdivide.insert(he_id, len_sqr);\n                }\n            }\n        }\n\n        #[cfg(feature = \"rerun\")]\n        self.log_rerun();\n    }\n",
            "                    halfedges_to_subdivide.insert(he_id, len_sqr);\n                }\n            }\n\n            cascade_iter += 1;\n            if cascade_iter % 500 == 0 {\n                super::cascade_check::log_op(self, &format!(\"subdivide iter {}\", cascade_iter), Some(max_length_squared), None, &format!(\"set={}\", halfedges_to_subdivide.len()));\n            }\n        }\n\n        super::cascade_check::log_op(self, \"subdivide exit\", Some(max_length_squared), None, \"\");\n        super::cascade_check::log_topo(self, \"subdivide exit\");\n\n        #[cfg(feature = \"rerun\")]\n        self.log_rerun();\n    }\n",
            1,
        ),
    ], "subdivide.rs")

    # -- collapse.rs ---------------------------------------------------------
    patch(f"{ops}/collapse.rs", [
        (
            "        let mut halfedges_to_collapse = self.halfedges_map(|len_sqr| len_sqr < min_length_squared);\n",
            "        let mut halfedges_to_collapse = self.halfedges_map(|len_sqr| len_sqr < min_length_squared);\n\n        super::cascade_check::log_op(self, \"collapse entry\", None, Some(min_length_squared), &format!(\"set={}\", halfedges_to_collapse.len()));\n        super::cascade_check::log_topo(self, \"collapse entry\");\n\n        // Bounded work: a degenerate region can keep re-feeding the set forever.\n        let budget = halfedges_to_collapse.len() * 2 + 100;\n",
            1,
        ),
        (
            "        for _ in 0..self.halfedges.len() {\n            if halfedges_to_collapse.is_empty() {",
            "        let mut cascade_iter = 0usize;\n\n        for _ in 0..budget {\n            if halfedges_to_collapse.is_empty() {",
            1,
        ),
        (
            "                    halfedges_to_collapse.remove(&he_id);\n                }\n            }\n        }\n",
            "                    halfedges_to_collapse.remove(&he_id);\n                }\n            }\n\n            cascade_iter += 1;\n            if cascade_iter % 500 == 0 {\n                super::cascade_check::log_op(self, &format!(\"collapse iter {}\", cascade_iter), None, Some(min_length_squared), &format!(\"set={}\", halfedges_to_collapse.len()));\n            }\n        }\n",
            1,
        ),
        (
            "        self.rebuild_outgoing_halfedges();\n\n        #[cfg(feature = \"rerun\")]",
            "        self.rebuild_outgoing_halfedges();\n\n        super::cascade_check::log_op(self, \"collapse exit\", None, Some(min_length_squared), \"\");\n        super::cascade_check::log_topo(self, \"collapse exit\");\n\n        #[cfg(feature = \"rerun\")]",
            1,
        ),
    ], "collapse.rs")

    # -- merge_one_ring/mod.rs ----------------------------------------------
    patch(f"{ops}/merge_one_ring/mod.rs", [
        (
            "        let mut result = MergeVerticesOneRing::default();\n",
            "        let mut result = MergeVerticesOneRing::default();\n\n        super::cascade_check::log_op(self, \"merge entry\", None, Some(flip_threshold_sqr), \"\");\n        super::cascade_check::log_topo(self, \"merge entry\");\n\n        let ring1: Vec<crate::HalfedgeId> = self.vertices.get(vertex_id1).map(|v| v.one_ring(self).collect()).unwrap_or_default();\n        let ring2: Vec<crate::HalfedgeId> = self.vertices.get(vertex_id2).map(|v| v.one_ring(self).collect()).unwrap_or_default();\n        tracing::info!(target: \"mesh_graph::ops::cascade_check\", \"merge rings: v1={vertex_id1:?} ring1={ring1:?}\\nv2={vertex_id2:?} ring2={ring2:?}\");\n",
            1,
        ),
        (
            "        self.smooth_vertices(\n            one_ring_v_ids1\n                .iter()\n                .chain(&one_ring_v_ids2)\n                .chain(&result.added_vertices)\n                .copied(),\n        );\n\n        result\n    }",
            "        self.smooth_vertices(\n            one_ring_v_ids1\n                .iter()\n                .chain(&one_ring_v_ids2)\n                .chain(&result.added_vertices)\n                .copied(),\n        );\n\n        super::cascade_check::log_op(\n            self,\n            \"merge exit\",\n            None,\n            Some(flip_threshold_sqr),\n            &format!(\n                \"rem_v={} rem_he={} rem_f={} add_v={} add_he={} add_f={}\",\n                result.removed_vertices.len(),\n                result.removed_halfedges.len(),\n                result.removed_faces.len(),\n                result.added_vertices.len(),\n                result.added_halfedges.len(),\n                result.added_faces.len(),\n            ),\n        );\n        super::cascade_check::log_topo(self, \"merge exit\");\n\n        result\n    }",
            1,
        ),
    ], "merge_one_ring/mod.rs")

    # -- merge_one_ring/mod.rs: repair_face_pointers before smooth_vertices ----
    patch(f"{ops}/merge_one_ring/mod.rs", [
        (
            "        self.smooth_vertices(\n            one_ring_v_ids1",
            "        self.repair_face_pointers();\n\n        self.smooth_vertices(\n            one_ring_v_ids1",
            1,
        ),
    ], "merge_one_ring/mod.rs (repair)")

    # -- merge_one_ring/mod.rs: skip degenerate planned faces (item-3 fix) ----
    patch(f"{ops}/merge_one_ring/mod.rs", [
        (
            "        for other_idx in others {\n            let other_v_id = other_ids[other_idx % other_ids.len()];\n\n            if other_v_id == single_v_id {\n                break;\n            }\n\n            // make sure the triangle vertices are CCW\n",
            "        for other_idx in others {\n            let other_v_id = other_ids[other_idx % other_ids.len()];\n\n            if other_v_id == single_v_id {\n                break;\n            }\n\n            if other_v_id == prev_other_v_id {\n                // Consecutive duplicates in the \"other\" ring would produce a\n                // degenerate (V, S, V) self-connected face. Skip the face but\n                // keep advancing so the following triangles stay well-formed.\n                prev_other_v_id = other_v_id;\n                continue;\n            }\n\n            // make sure the triangle vertices are CCW\n",
            1,
        ),
        (
            "    fn add_to_mesh_graph(\n        &self,\n        mesh_graph: &mut MeshGraph,\n    ) -> Option<(Option<HalfedgeId>, AddFace)> {\n        #[cfg(feature = \"rerun\")]\n        self.log_rerun(\"add_to_mesh_graph\", mesh_graph);\n\n        let add_or_get_edge1 = mesh_graph.add_or_get_boundary_edge(self.v1, self.new_he_v1)?;\n",
            "    fn add_to_mesh_graph(\n        &self,\n        mesh_graph: &mut MeshGraph,\n    ) -> Option<(Option<HalfedgeId>, AddFace)> {\n        #[cfg(feature = \"rerun\")]\n        self.log_rerun(\"add_to_mesh_graph\", mesh_graph);\n\n        if self.v1 == self.new_he_v1 || self.v1 == self.new_he_v2 || self.new_he_v1 == self.new_he_v2 {\n            // A planned face with a repeated vertex creates a self-loop edge and\n            // zero-area geometry; it must never reach the mesh.\n            tracing::error!(\n                \"Skipping degenerate planned face with repeated vertices: {:?}\",\n                (self.v1, self.new_he_v1, self.new_he_v2)\n            );\n            return None;\n        }\n\n        let add_or_get_edge1 = mesh_graph.add_or_get_boundary_edge(self.v1, self.new_he_v1)?;\n",
            1,
        ),
        (
            "    fn add_to_mesh_graph_and_he(\n        &self,\n        mesh_graph: &mut MeshGraph,\n        existing_he_id: HalfedgeId,\n    ) -> Option<(Option<HalfedgeId>, AddFace)> {\n        #[cfg(feature = \"rerun\")]\n        self.log_rerun(\"add_to_mesh_graph_and_he\", mesh_graph);\n\n        match self.order {\n",
            "    fn add_to_mesh_graph_and_he(\n        &self,\n        mesh_graph: &mut MeshGraph,\n        existing_he_id: HalfedgeId,\n    ) -> Option<(Option<HalfedgeId>, AddFace)> {\n        #[cfg(feature = \"rerun\")]\n        self.log_rerun(\"add_to_mesh_graph_and_he\", mesh_graph);\n\n        if self.v1 == self.new_he_v1 || self.v1 == self.new_he_v2 || self.new_he_v1 == self.new_he_v2 {\n            tracing::error!(\n                \"Skipping degenerate planned face with repeated vertices: {:?}\",\n                (self.v1, self.new_he_v1, self.new_he_v2)\n            );\n            return None;\n        }\n\n        match self.order {\n",
            1,
        ),
    ], "merge_one_ring/mod.rs (degenerate-face guards)")

    # -- lib.rs: repair_face_pointers + rebuild pointer normalization ----------
    patch(f"{root}/src/lib.rs", [
        (
            "use slotmap::{SecondaryMap, SlotMap};",
            "use slotmap::{SecondaryMap, SlotMap};\n\nuse crate::elements::FaceId;",
            1,
        ),
        (
            "    pub fn rebuild_outgoing_halfedges(&mut self) {",
            "    /// Repairs the redundant `Halfedge::face` cache from the halfedge chains, which are\n    /// the ground truth for face membership (just like `rebuild_outgoing_halfedges` is for\n    /// the per-vertex lists). Stale `.face` pointers (e.g. after flap-removal twin\n    /// re-pairs) make later operations subdivide the wrong faces and degenerate the mesh.\n    /// Also clears `.face` on halfedges that are no longer reachable from any face's chain.\n    ///\n    /// O(F + H), meant to be called once per operation that rewires faces.\n    pub fn repair_face_pointers(&mut self) {\n        let face_ids: Vec<FaceId> = self.faces.keys().collect();\n        let mut visited: hashbrown::HashMap<HalfedgeId, FaceId> = hashbrown::HashMap::new();\n\n        for face_id in face_ids {\n            let Some(start_he) = self.faces.get(face_id).map(|f| f.halfedge) else {\n                continue;\n            };\n\n            let mut he_id = start_he;\n            for _ in 0..32 {\n                let Some(he) = self.halfedges.get_mut(he_id) else {\n                    break;\n                };\n                he.face = Some(face_id);\n                visited.insert(he_id, face_id);\n\n                let Some(next) = he.next else {\n                    break;\n                };\n                if next == start_he {\n                    break;\n                }\n                he_id = next;\n            }\n        }\n\n        // Halfedges that claim a face but are not reachable from that face's chain are\n        // orphans left behind by re-links (e.g. flap twin re-pairs). Clear their stale\n        // `.face` so they read as boundary, which all traversals handle.\n        let orphan_ids: Vec<HalfedgeId> = self\n            .halfedges\n            .iter()\n            .filter(|(he_id, he)| he.face.is_some() && !visited.contains_key(he_id))\n            .map(|(he_id, _)| he_id)\n            .collect();\n\n        for he_id in orphan_ids {\n            if let Some(he) = self.halfedges.get_mut(he_id) {\n                he.face = None;\n            }\n        }\n    }\n\n    pub fn rebuild_outgoing_halfedges(&mut self) {",
            1,
        ),
    ], "lib.rs (repair_face_pointers)")

    # -- lib.rs: rebuild_outgoing_halfedges pointer normalization ------------
    patch(f"{root}/src/lib.rs", [
        (
            "            entry.or_default().push(twin_id);\n        }\n    }\n",
            "            entry.or_default().push(twin_id);\n        }\n\n        // Normalize the per-vertex seed pointers (`vertices[v].outgoing_halfedge`). Stale\n        // seeds pointing at removed halfedges make ring traversals (`one_ring`, faces, ...)\n        // yield dead ids, which panics callers that index them. Only replace the seed when\n        // it is dead, to keep ring iteration order stable in the common case.\n        for (v_id, vertex) in &mut self.vertices {\n            let stored_seed = vertex.outgoing_halfedge;\n            let live_seed = stored_seed.filter(|he| self.halfedges.contains_key(*he));\n            vertex.outgoing_halfedge = live_seed.or_else(|| {\n                self.outgoing_halfedges.get(v_id).and_then(|list| list.first().copied())\n            });\n        }\n    }\n",
            1,
        ),
    ], "lib.rs")

    print("ALL PATCHED")

if __name__ == "__main__":
    main(sys.argv[1])