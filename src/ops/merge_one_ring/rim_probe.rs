//! Test-only collector for the boundary-delta taxonomy of a single merge.
//!
//! The op pushes one [`RimProbe`] per band merge (thread-local, so parallel
//! tests do not interleave). Probes are measurements, not assertions: Phase 0
//! records how far the current rebuild is from "closes exactly what it opened"
//! before Phase 1/2 make that structural.

use std::cell::{Cell, RefCell};

use hashbrown::HashSet;

use crate::{HalfedgeId, MeshGraph, VertexId};

use super::rim;
use super::rim::RimStep;

/// `before` and `rim` are filled in by the op as the removals happen;
/// `classify` walks the rim right before the planned faces go in, `finish`
/// computes the outcome once they are.
#[derive(Debug, Clone, Default)]
pub(crate) struct RimProbe {
    /// Boundary halves immediately before the first removal.
    pub before: HashSet<HalfedgeId>,
    /// `opened_rim` deltas of the removal batches (stars + strips).
    pub rim: HashSet<HalfedgeId>,
    /// Rim halves opened by removing the first merged vertex's star.
    pub h1: HashSet<HalfedgeId>,
    /// Rim halves opened by removing the second merged vertex's star (i.e. the
    /// rim delta of the second removal that is not the first's).
    pub h2: HashSet<HalfedgeId>,
    /// Composition of the unconsumed rim, one entry per loop/chain walk.
    pub unconsumed_shape: String,
    /// Ordered walk of the full opened rim `H`, one entry per half: side, half,
    /// start -> end, and whether the half was already boundary before the op.
    /// Only filled when `MESH_GRAPH_RIM_TAXONOMY` is set.
    pub rim_walk_detail: Vec<String>,
    /// Read-only measurement of the rim-first planner on this rim: faces it
    /// would emit, how many rim halves they consume, and what it could not
    /// plan. Independent of whether the op actually used the plan.
    pub proto_faces: usize,
    pub proto_consumed: usize,
    pub proto_unhandled: Vec<String>,
    /// Per-structure run composition of the rim, for diagnostics.
    pub proto_structure_detail: Vec<String>,
    /// Lengths of the ordered ring each star's rim forms. The old planner ran
    /// on the pre-removal one-rings, which include vertices that are *not* on
    /// the rim (strip-connected and hole-interior ones); these lengths are the
    /// rim-only truth Phase 2's planner must consume.
    pub h1_ring_len: usize,
    pub h2_ring_len: usize,

    /// Closed rim loops (walk returns to its start).
    pub rim_loops: usize,
    /// Rim chains ending on a pre-existing border.
    pub rim_chains: usize,
    /// Rim halves whose walk is broken (isolated or non-manifold).
    pub rim_bad: usize,
    /// Details of the broken walks, for diagnostics.
    pub rim_bad_details: Vec<String>,
    /// Star-rim halves that the strip removal freed again (their edge vanished
    /// entirely), so they are not part of the live rim.
    pub rim_freed: usize,

    /// Rim halves that are still boundary after the planned faces were added.
    pub unconsumed: HashSet<HalfedgeId>,
    /// Edges whose two halves are both face-less after the planned faces.
    pub wire: Vec<HalfedgeId>,
    /// Boundary halves that are neither in `before` nor in `rim`.
    pub new_boundary: HashSet<HalfedgeId>,
    /// Planned faces the insertion skipped (validation, refused acquisition, ...).
    pub skipped_faces: usize,
    /// Distinct vertex pairs joined by more than one halfedge from the same
    /// start vertex, mesh-wide after the insertion (parallel leftovers).
    pub parallel_edges: usize,
    /// The same count before the op started, so the delta is what this merge
    /// changed (captured regions can contain pre-existing parallels).
    pub parallel_before: usize,

    /// End-of-op numbers, after `cleanup_bookkeeping`, the remaining-unconnected
    /// merge and the repair passes. The pre-net sample above is where the
    /// planner's own behavior is visible; this is where the op's exit state is.
    pub end_unconsumed: usize,
    pub end_new_boundary: usize,
    /// End-of-op wire halves. This covers only what the *merge op* leaves
    /// behind; wire created between merges (e.g. by the collapse/subdivide
    /// cleanup in freestyle's `cleanup_mesh`) is outside this probe and remains
    /// an open item tracked in the weld handoff.
    pub end_wire: Vec<HalfedgeId>,
    pub end_parallel_edges: usize,
}

/// Summarize the loops/chains of `halves`, with their side composition.
fn describe_structures(
    mesh: &MeshGraph,
    halves: &HashSet<HalfedgeId>,
    h1: &HashSet<HalfedgeId>,
    h2: &HashSet<HalfedgeId>,
) -> String {
    if halves.is_empty() {
        return String::new();
    }

    let mut starts: Vec<HalfedgeId> = halves.iter().copied().collect();
    starts.sort_unstable();

    let mut has_predecessor = HashSet::new();
    for &he_id in &starts {
        if let RimStep::Continue(next) = rim::next_rim_half(mesh, he_id, halves) {
            has_predecessor.insert(next);
        }
    }

    let mut visited = HashSet::new();
    let mut parts = Vec::new();

    let mut walk = |start: HalfedgeId, visited: &mut HashSet<HalfedgeId>| {
        let mut current = start;
        let mut count = (0usize, 0usize, 0usize);
        let mut length = 0usize;
        let terminal;
        loop {
            length += 1;
            if h1.contains(&current) {
                count.0 += 1;
            } else if h2.contains(&current) {
                count.1 += 1;
            } else {
                count.2 += 1;
            }
            match rim::next_rim_half(mesh, current, halves) {
                RimStep::Continue(next) => {
                    if !visited.insert(next) {
                        terminal = if next == start { "loop" } else { "broken" };
                        break;
                    }
                    current = next;
                }
                RimStep::ChainEnd => {
                    terminal = "chain";
                    break;
                }
                RimStep::Isolated => {
                    terminal = "isolated";
                    break;
                }
            }
        }
        parts.push(format!(
            "{terminal} {length} h1={} h2={} other={}",
            count.0, count.1, count.2
        ));
    };

    for &start in starts.iter().filter(|he| !has_predecessor.contains(*he)) {
        if visited.insert(start) {
            walk(start, &mut visited);
        }
    }
    for &start in &starts {
        if visited.insert(start) {
            walk(start, &mut visited);
        }
    }

    format!("{} struct(s): {}", parts.len(), parts.join(", "))
}

/// Mesh-wide count of parallel halfedges: directed vertex pairs `(v, u)` with
/// more than one halfedge from `v` to `u`.
pub(crate) fn count_parallel_edges(mesh: &MeshGraph) -> usize {
    let mut seen = HashSet::<(VertexId, VertexId)>::new();
    let mut parallel_edges = 0;
    for (v_id, out_hes) in mesh.outgoing_halfedges.iter() {
        for he_id in out_hes {
            if let Some(he) = mesh.halfedges.get(*he_id)
                && !seen.insert((v_id, he.end_vertex))
            {
                parallel_edges += 1;
            }
        }
    }
    parallel_edges
}

impl RimProbe {
    /// Walk the rim with [`rim::next_rim_half`] and count loops, border-ending
    /// chains and broken starts. Call while all rim halves are live (before the
    /// planned faces are added).
    pub(crate) fn classify(&mut self, mesh: &MeshGraph) {
        // A halfedge the star removal opened can be freed again by the strip
        // removal (both halves of its edge belonged to a removed face). Those
        // are no longer part of the rim; keep them only as a count.
        let before_filter = self.rim.len();
        self.rim.retain(|he_id| mesh.halfedges.contains_key(*he_id));
        self.rim_freed = before_filter - self.rim.len();

        self.h1.retain(|he_id| mesh.halfedges.contains_key(*he_id));
        self.h2.retain(|he_id| mesh.halfedges.contains_key(*he_id));

        let mut starts: Vec<HalfedgeId> = self.rim.iter().copied().collect();
        starts.sort_unstable();

        // A rim half whose next rim half exists has a predecessor. Chains must
        // be walked from their heads (no predecessor) first: walking them from
        // an ascending seed that happens to land mid-chain makes the walk enter
        // an already-visited half and would score a legitimate chain as
        // `rim_bad` (the Phase 0/1 review caught exactly that). Whatever is
        // left after the head pass is loops (and anything a head pass could not
        // reach).
        let mut has_predecessor = HashSet::new();
        for &he_id in &starts {
            if let RimStep::Continue(next) = rim::next_rim_half(mesh, he_id, &self.rim) {
                has_predecessor.insert(next);
            }
        }

        let mut visited = HashSet::new();

        for &start in starts.iter().filter(|he| !has_predecessor.contains(*he)) {
            if !visited.insert(start) {
                continue;
            }
            self.walk_rim(mesh, start, &mut visited);
        }

        for &start in &starts {
            if !visited.insert(start) {
                continue;
            }
            self.walk_rim(mesh, start, &mut visited);
        }

        if std::env::var("MESH_GRAPH_RIM_TAXONOMY").is_ok() {
            self.record_walk_detail(mesh, &starts);
        }
    }

    /// Dump the ordered rim walk (head-first, ascending ids) with per-half side
    /// and endpoints. Diagnostics only; the planner must not depend on this.
    fn record_walk_detail(&mut self, mesh: &MeshGraph, starts: &[HalfedgeId]) {
        let mut has_predecessor = HashSet::new();
        for &he_id in starts {
            if let RimStep::Continue(next) = rim::next_rim_half(mesh, he_id, &self.rim) {
                has_predecessor.insert(next);
            }
        }

        let side_of = |he_id: &HalfedgeId| -> &'static str {
            if self.h1.contains(he_id) {
                "h1"
            } else if self.h2.contains(he_id) {
                "h2"
            } else {
                "strip"
            }
        };
        let describe = |mesh: &MeshGraph, he_id: HalfedgeId| -> String {
            let he = mesh.halfedges.get(he_id);
            let start = he.and_then(|he| he.start_vertex(mesh));
            let end = he.map(|he| he.end_vertex);
            format!(
                "{he_id:?} {} {start:?}->{end:?}{}",
                side_of(&he_id),
                if self.before.contains(&he_id) {
                    " (pre-existing)"
                } else {
                    ""
                }
            )
        };

        let mut visited = HashSet::new();
        let walk = |start: HalfedgeId, visited: &mut HashSet<HalfedgeId>, out: &mut Vec<String>| {
            let mut current = start;
            let mut chain = Vec::new();
            loop {
                chain.push(describe(mesh, current));
                match rim::next_rim_half(mesh, current, &self.rim) {
                    RimStep::Continue(next) => {
                        if !visited.insert(next) {
                            out.push(format!(
                                "{} [{}]",
                                if next == start { "loop" } else { "broken" },
                                chain.join(" | ")
                            ));
                            return;
                        }
                        current = next;
                    }
                    RimStep::ChainEnd => {
                        out.push(format!("chain-end [{}]", chain.join(" | ")));
                        return;
                    }
                    RimStep::Isolated => {
                        out.push(format!("isolated [{}]", chain.join(" | ")));
                        return;
                    }
                }
            }
        };

        for &start in starts.iter().filter(|he| !has_predecessor.contains(*he)) {
            if visited.insert(start) {
                walk(start, &mut visited, &mut self.rim_walk_detail);
            }
        }
        for &start in starts {
            if visited.insert(start) {
                walk(start, &mut visited, &mut self.rim_walk_detail);
            }
        }
    }

    /// One full walk from `start`; updates the loop/chain/bad counters.
    fn walk_rim(&mut self, mesh: &MeshGraph, start: HalfedgeId, visited: &mut HashSet<HalfedgeId>) {
        let mut current = start;
        loop {
            match rim::next_rim_half(mesh, current, &self.rim) {
                RimStep::Continue(next) => {
                    if !visited.insert(next) {
                        if next == start {
                            self.rim_loops += 1;
                        } else {
                            self.rim_bad += 1;
                            self.rim_bad_details.push(format!(
                                "{current:?} -> {next:?}: already visited (non-manifold rim)"
                            ));
                        }
                        return;
                    }
                    current = next;
                }
                RimStep::ChainEnd => {
                    self.rim_chains += 1;
                    return;
                }
                RimStep::Isolated => {
                    self.rim_bad += 1;
                    let end_v = mesh.halfedges.get(current).map(|he| he.end_vertex);
                    self.rim_bad_details
                        .push(format!("{current:?} -> {end_v:?}: isolated"));
                    return;
                }
            }
        }
    }

    /// Measure the rim-first planner on the current rim. Read-only: the planned
    /// faces are never inserted, so the op's behavior is unchanged.
    /// Call after [`Self::classify`], when `rim`/`h1`/`h2` describe the live rim.
    pub(crate) fn measure_rim_plan(&mut self, mesh: &MeshGraph) {
        let structures = super::rim_planner::rim_structures(mesh, &self.rim, &self.h1, &self.h2);
        self.proto_structure_detail = structures
            .iter()
            .map(|s| {
                let runs: Vec<String> = s
                    .runs
                    .iter()
                    .map(|r| format!("{:?}/{}v", r.side, r.vertices.len()))
                    .collect();
                format!(
                    "{} runs [{}]",
                    if s.cyclic { "cyclic" } else { "open" },
                    runs.join(", ")
                )
            })
            .collect();

        let plan = super::rim_planner::plan(mesh, &self.rim, &self.h1, &self.h2);
        self.proto_faces = plan.faces.len();
        self.proto_consumed = plan.consumed_halves(mesh, &self.rim).len();
        self.proto_unhandled = plan.unhandled;
    }

    pub(crate) fn finish(&mut self, mesh: &MeshGraph) {
        let boundary_now = rim::boundary_halves(mesh);

        self.unconsumed = self
            .rim
            .iter()
            .copied()
            .filter(|he_id| boundary_now.contains(he_id))
            .collect();

        self.unconsumed_shape = describe_structures(mesh, &self.unconsumed, &self.h1, &self.h2);
        self.h1_ring_len = super::rim::ring_from_halves(mesh, &self.h1).0.len();
        self.h2_ring_len = super::rim::ring_from_halves(mesh, &self.h2).0.len();

        self.wire = boundary_now
            .iter()
            .copied()
            // Pre-existing wire edges are not this merge's doing.
            .filter(|he_id| !self.before.contains(he_id))
            .filter(|he_id| {
                mesh.halfedges
                    .get(*he_id)
                    .and_then(|he| he.twin)
                    .and_then(|twin_id| mesh.halfedges.get(twin_id))
                    .is_some_and(|twin| twin.face.is_none())
            })
            .collect();
        self.wire.sort();

        self.new_boundary = boundary_now
            .into_iter()
            .filter(|he_id| !self.before.contains(he_id) && !self.rim.contains(he_id))
            .collect();

        self.parallel_edges = count_parallel_edges(mesh);
    }

    /// End-of-op measurement. The caller must have run [`Self::finish`] first.
    pub(crate) fn finish_end(&mut self, mesh: &MeshGraph) {
        let boundary_now = rim::boundary_halves(mesh);

        self.end_unconsumed = self
            .rim
            .iter()
            .copied()
            .filter(|he_id| boundary_now.contains(he_id))
            .count();
        self.end_new_boundary = boundary_now
            .iter()
            .copied()
            .filter(|he_id| !self.before.contains(he_id) && !self.rim.contains(he_id))
            .count();

        self.end_wire = boundary_now
            .iter()
            .copied()
            .filter(|he_id| {
                mesh.halfedges
                    .get(*he_id)
                    .and_then(|he| he.twin)
                    .and_then(|twin_id| mesh.halfedges.get(twin_id))
                    .is_some_and(|twin| twin.face.is_none())
            })
            .collect();
        self.end_wire.sort();

        self.end_parallel_edges = count_parallel_edges(mesh);
    }

    /// A stable, printable one-line summary.
    pub(crate) fn summary(&self) -> String {
        let count = |set: &HashSet<HalfedgeId>| set.len();
        let wire: Vec<String> = self.wire.iter().map(|he_id| format!("{he_id:?}")).collect();
        let mut summary = format!(
            "before {} rim {} (rings {}v/{}v loops {} chains {} bad {} freed {} {:?}) -> unconsumed {} new_boundary {} wire {:?} skipped {} parallel {}->{} end (unconsumed {} new_boundary {} wire {} parallel {})",
            self.before.len(),
            self.rim.len(),
            self.h1_ring_len,
            self.h2_ring_len,
            self.rim_loops,
            self.rim_chains,
            self.rim_bad,
            self.rim_freed,
            self.rim_bad_details,
            count(&self.unconsumed),
            count(&self.new_boundary),
            wire,
            self.skipped_faces,
            self.parallel_before,
            self.parallel_edges,
            self.end_unconsumed,
            self.end_new_boundary,
            self.end_wire.len(),
            self.end_parallel_edges,
        );
        if !self.unconsumed_shape.is_empty() {
            summary.push_str(&format!(" shape [{}]", self.unconsumed_shape));
        }
        summary.push_str(&format!(
            " proto (faces {} consumed {}/{}{})",
            self.proto_faces,
            self.proto_consumed,
            self.rim.len(),
            if self.proto_unhandled.is_empty() {
                String::new()
            } else {
                format!(" unhandled {:?}", self.proto_unhandled)
            }
        ));
        if !self.proto_structure_detail.is_empty() {
            summary.push_str(&format!(" structs {:?}", self.proto_structure_detail));
        }
        summary
    }
}

thread_local! {
    static RECORDS: RefCell<Vec<RimProbe>> = const { RefCell::new(Vec::new()) };
    static SKIPPED_FACES: Cell<usize> = const { Cell::new(0) };
}

/// Record that a planned face was skipped. Read back by the next [`push`].
pub(crate) fn note_skipped_face() {
    SKIPPED_FACES.with(|skipped| skipped.set(skipped.get() + 1));
}

pub(crate) fn push(mut probe: RimProbe) {
    probe.skipped_faces = SKIPPED_FACES.with(|skipped| skipped.replace(0));
    RECORDS.with(|records| records.borrow_mut().push(probe));
}

pub(crate) fn drain() -> Vec<RimProbe> {
    RECORDS.with(|records| records.borrow_mut().drain(..).collect())
}
