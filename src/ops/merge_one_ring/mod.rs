#[cfg(test)]
mod rim;
#[cfg(test)]
mod rim_probe;
#[cfg(test)]
mod tests;

use std::ops::RangeInclusive;

use hashbrown::{HashMap, HashSet};
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    FaceId, HalfedgeId, MeshGraph, VertexId, error_none,
    ops::{add::AddFace, collapse::CollapseEdge, edit::MergeVertices},
    utils::unwrap_or_return,
};

#[derive(Default)]
pub struct MergeVerticesOneRing {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    pub added_vertices: Vec<VertexId>,
    pub added_halfedges: Vec<HalfedgeId>,
    pub added_faces: Vec<FaceId>,
}

/// The three post-hoc repair passes of the merge ("the nets"):
/// `weld_parallel_edges` (before and after the neighborhood cleanup) and
/// `close_triangular_holes`.
///
/// Enabled by default; production behavior is unchanged (the non-test branch
/// folds to `true`). Test builds can turn them off with
/// `MESH_GRAPH_MERGE_NETS=0`, which is the rim-first rebuild's oracle: with the
/// nets off, the corpus measures whether the rebuild itself closes exactly what
/// the removals opened, instead of whether the nets mop up afterwards.
///
/// Phase 0 baseline with the nets off: `parallel_edge_leftover` and
/// `weld_cases_stay_watertight` fail; the rest of the corpus passes.
#[cfg(test)]
fn merge_nets_enabled() -> bool {
    use std::sync::OnceLock;

    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var("MESH_GRAPH_MERGE_NETS").map_or(true, |value| value != "0" && value != "off")
    })
}

#[cfg(not(test))]
#[inline(always)]
fn merge_nets_enabled() -> bool {
    true
}

impl MeshGraph {
    /// Merge two vertices by connecting their 1-rings.
    ///
    /// The vertices are deleted on top of everything that is returned in the `removed_...` fields.
    ///
    /// See [Freestyle: Sculpting meshes with self-adaptive topology DOI 10.1016/j.cag.2011.03.033](https://inria.hal.science/inria-00606516v1/document)
    /// Chapters 3.2 and 5.1
    #[instrument(skip(self, marked_halfedges, marked_vertices))]
    pub fn merge_vertices_one_rings(
        &mut self,
        vertex_id1: VertexId,
        vertex_id2: VertexId,
        flip_threshold_sqr: f32,
        marked_halfedges: &mut HashSet<HalfedgeId>,
        marked_vertices: &mut HashSet<VertexId>,
    ) -> MergeVerticesOneRing {
        #[cfg(feature = "instrumentation")]
        crate::set_current_op("merge_one_ring");
        #[cfg(feature = "instrumentation")]
        crate::probe_chain_begin(self);
        let mut result = MergeVerticesOneRing::default();

        let vertex1 = *unwrap_or_return!(self.vertices.get(vertex_id1), "Vertex not found", result);
        let vertex2 = *unwrap_or_return!(self.vertices.get(vertex_id2), "Vertex not found", result);

        if vertex_id1 == vertex_id2 {
            error!("Cannot merge a vertex with itself");
            return result;
        }

        let one_ring_he_ids1 = vertex1.one_ring(self).collect_vec();
        let mut one_ring_he_ids2 = vertex2
            .one_ring(self)
            // halfedges are reversed, so twin halfedges are actually the next halfedges.
            // also existence is checked in `one_ring()`.
            .filter_map(|he_id| {
                // `one_ring` derives the ids from the outgoing star of the vertex via
                // twins; a dead id here (stale seed or dangling twin) would panic on
                // the index below, so skip it defensively.
                let he = self.halfedges.get(he_id)?;
                he.twin.or_else(error_none!("Twin not found"))
            })
            .collect_vec();
        one_ring_he_ids2.reverse();

        if one_ring_he_ids1.len() < 3 || one_ring_he_ids2.len() < 3 {
            error!(
                "One rings are too small. One ring of {vertex_id1:?} = {}; one ring of {vertex_id2:?} = {}",
                one_ring_he_ids1.len(),
                one_ring_he_ids2.len()
            );
            return result;
        }

        // halfedges' existence already checked in `one_ring()`.
        let mut one_ring_v_ids1 = one_ring_he_ids1
            .iter()
            .filter_map(|he_id| {
                // `one_ring` derives the ids from the outgoing star of the vertex via
                // twins; a dead id here (stale seed or dangling twin) would panic on
                // the index below, so skip it defensively.
                if self.halfedges.contains_key(*he_id) {
                    Some(self.halfedges[*he_id].end_vertex)
                } else {
                    None
                }
            })
            .collect_vec();
        one_ring_v_ids1.rotate_right(1);
        let mut one_ring_v_ids2 = one_ring_he_ids2
            .iter()
            .filter_map(|he_id| {
                if self.halfedges.contains_key(*he_id) {
                    Some(self.halfedges[*he_id].end_vertex)
                } else {
                    None
                }
            })
            .collect_vec();
        one_ring_v_ids2.rotate_right(1);

        if one_ring_v_ids1.len() < 3 || one_ring_v_ids2.len() < 3 {
            error!("One rings are too small");
            return result;
        }

        // Phase 0 rim taxonomy: measure the boundary the op opens and closes.
        // Test-only; records one probe per band merge into a thread-local.
        #[cfg(test)]
        let mut test_rim_probe = rim_probe::RimProbe::default();
        #[cfg(test)]
        {
            test_rim_probe.before = rim::boundary_halves(self);
            test_rim_probe.parallel_before = rim_probe::count_parallel_edges(self);
        }

        // Weld position-coincident cross-ring vertices before planning the band.
        //
        // When the two sheets touch (morphological close / collision merge), a
        // ring vertex of `vertex_id1` and a ring vertex of `vertex_id2` can share
        // a position while remaining distinct ids. Both rings then pass through
        // the same point, and any band spanning that zero-width seam contains two
        // faces over the same geometric triangle — a "flap" by the position-based
        // `faces_share_all_vertices`. `cleanup_bookkeeping` deletes the flap,
        // leaving a hole that nothing re-fills (the intermittent weld-hole
        // defect, ~1 in 12 runs, boundary halfedges left open).
        //
        // Fusing each coincident pair turns the two rings into the well-tested
        // shared-vertex case: the band is split at the shared vertex and the seam
        // is never spanned by a degenerate face. The position comparison is exact,
        // matching `faces_share_all_vertices`, so this fires exactly where the
        // flap deletion would have.
        let mut coincident_pairs = Vec::new();
        for &v_id1 in &one_ring_v_ids1 {
            for &v_id2 in &one_ring_v_ids2 {
                if v_id1 != v_id2
                    && self
                        .positions
                        .get(v_id1)
                        .is_some_and(|p1| self.positions.get(v_id2).is_some_and(|p2| p1 == p2))
                {
                    coincident_pairs.push((v_id1, v_id2));
                }
            }
        }
        for (v1_id, v2_id) in &coincident_pairs {
            tracing::debug!("fusing coincident ring vertices {v1_id:?} and {v2_id:?}");
            // Survivor is the ring-1 vertex, so the fused id stays on both rings
            // and shows up in `shared_v_ids` below.
            self.merge_vertices([*v1_id, *v2_id]);
            result.removed_vertices.push(*v2_id);
        }
        if !coincident_pairs.is_empty() {
            let fused: HashMap<VertexId, VertexId> = coincident_pairs
                .iter()
                .map(|(survivor, dead)| (*dead, *survivor))
                .collect();
            for v_id in &mut one_ring_v_ids2 {
                if let Some(survivor) = fused.get(v_id) {
                    *v_id = *survivor;
                }
            }
        }

        let one_ring_he_set1 = HashSet::<HalfedgeId>::from_iter(one_ring_he_ids1.iter().copied());
        let one_ring_he_set2 = HashSet::<HalfedgeId>::from_iter(one_ring_he_ids2.iter().copied());
        let shared_he_ids = HashSet::<HalfedgeId>::from_iter(
            one_ring_he_set1.intersection(&one_ring_he_set2).copied(),
        );

        let one_ring_v_set1 = HashSet::<VertexId>::from_iter(one_ring_v_ids1.iter().copied());
        let one_ring_v_set2 = HashSet::<VertexId>::from_iter(one_ring_v_ids2.iter().copied());
        let shared_v_ids =
            HashSet::<VertexId>::from_iter(one_ring_v_set1.intersection(&one_ring_v_set2).copied());

        tracing::debug!("ring1 (v_ids): {one_ring_v_ids1:?}");
        tracing::debug!("ring2 (v_ids): {one_ring_v_ids2:?}");
        tracing::debug!("shared_v_ids: {shared_v_ids:?}");

        if self.check_and_flip_single_shared_he(&shared_he_ids, flip_threshold_sqr, &mut result) {
            return self.finish_merge(result);
        }

        self.remove_vertex_faces(vertex_id1, &mut result);

        #[cfg(test)]
        {
            test_rim_probe.h1 = rim::opened_rim(self, &test_rim_probe.before);
        }

        self.remove_vertex_faces(vertex_id2, &mut result);

        #[cfg(test)]
        {
            test_rim_probe
                .rim
                .extend(rim::opened_rim(self, &test_rim_probe.before));
            let h1 = test_rim_probe.h1.clone();
            test_rim_probe.h2 = test_rim_probe.rim.difference(&h1).copied().collect();
        }

        #[cfg(feature = "rerun")]
        self.log_rerun();

        let (already_connected_face_ids, connected_v_ids, connected_he_ids) = match self
            .find_already_connected_pairings(&one_ring_v_ids1, &one_ring_v_ids2, &shared_v_ids)
        {
            Some(pairings) => pairings,
            None => {
                tracing::error!("Error in find_already_connected_pairings");
                return self.finish_merge(result);
            }
        };

        #[cfg(feature = "rerun")]
        {
            self.log_faces_rerun("already_connected", &already_connected_face_ids);
            self.log_hes_rerun("already_connected", &connected_he_ids);
        }

        let range_pairs_to_connect = self.compute_range_pairs_to_connect(
            &one_ring_v_ids1,
            &one_ring_v_ids2,
            &one_ring_he_ids1,
            &one_ring_he_ids2,
            &shared_v_ids,
            &shared_he_ids,
            connected_v_ids,
            connected_he_ids,
        );

        // No band to build. That is a legitimate outcome - it means every ring
        // vertex is either shared with the other ring or already joined to it by
        // a face, so there is nothing left to span - but it must NOT end the op:
        // `remove_neighbor_faces` has already taken both stars out, and bailing
        // here leaves that hole with nothing to close it (two of the extracted
        // weld cases are exactly this). Fall through with nothing planned and let
        // the cleanup and the remaining-unconnected merge repair the region.
        if range_pairs_to_connect.is_empty() {
            tracing::debug!("No range pairs to connect; repairing the freed region instead");
        }

        tracing::debug!("Range pairs to connect: {range_pairs_to_connect:?}");

        let planned_faces =
            self.plan_new_faces(&range_pairs_to_connect, &one_ring_v_ids1, &one_ring_v_ids2);
        for pf in &planned_faces {
            tracing::debug!(
                "planned {:?} v1={:?} a={:?} b={:?}",
                pf.order,
                pf.v1,
                pf.new_he_v1,
                pf.new_he_v2
            );
        }

        // The already-connected strip is torn out to make room for the new faces.
        // With nothing planned there is nothing to make room for, so keep it -
        // removing it would only widen the hole.
        if !planned_faces.is_empty() {
            for face_id in already_connected_face_ids {
                let (v_ids, he_ids) = self.remove_face(face_id);
                result.removed_faces.push(face_id);
                result.removed_halfedges.extend(he_ids);
                result.removed_vertices.extend(v_ids);
            }
        }

        #[cfg(test)]
        {
            test_rim_probe
                .rim
                .extend(rim::opened_rim(self, &test_rim_probe.before));
            test_rim_probe.classify(self);
        }

        self.add_planned_faces(
            planned_faces,
            &one_ring_v_set1,
            &one_ring_v_set2,
            marked_halfedges,
            &mut result,
        );

        #[cfg(test)]
        {
            test_rim_probe.finish(self);
        }

        if merge_nets_enabled() {
            self.weld_parallel_edges(
                one_ring_v_ids1
                    .iter()
                    .chain(&one_ring_v_ids2)
                    .chain(&result.added_vertices)
                    .copied()
                    .collect_vec(),
                &mut result,
            );
        }

        #[cfg(feature = "rerun")]
        self.log_rerun();

        self.cleanup_bookkeeping(
            &one_ring_v_ids1,
            &one_ring_v_ids2,
            marked_vertices,
            marked_halfedges,
            &mut result,
        );

        self.merge_remaining_unconnected(
            one_ring_v_ids1.clone(),
            one_ring_v_ids2.clone(),
            &mut result,
        );

        // Again after the cleanup: a pair can still be parallel here that was not
        // weldable above, because the two faces hung off the *same* side of the
        // edge until the cleanup rebuilt one of them.
        if merge_nets_enabled() {
            self.weld_parallel_edges(
                one_ring_v_ids1
                    .iter()
                    .chain(&one_ring_v_ids2)
                    .chain(&result.added_vertices)
                    .copied()
                    .collect_vec(),
                &mut result,
            );

            self.close_triangular_holes(
                &one_ring_v_ids1
                    .iter()
                    .chain(&one_ring_v_ids2)
                    .chain(&result.added_vertices)
                    .copied()
                    .collect_vec(),
                &mut result,
            );
        }

        self.smooth_vertices(
            one_ring_v_ids1
                .iter()
                .chain(&one_ring_v_ids2)
                .chain(&result.added_vertices)
                .copied(),
        );

        #[cfg(test)]
        {
            test_rim_probe.finish_end(self);
            rim_probe::push(test_rim_probe);
        }

        self.finish_merge(result)
    }

    /// Common op-end path: run the integrity/hole inspectors and push the mesh
    /// onto the state-history ring (hunt instrumentation), then return the result.
    /// Every exit of the op goes through here (including the early failure
    /// exits), so the ring stays continuous with the journal (resume
    /// continuity) and no corruption can skip the checks.
    #[cfg(feature = "instrumentation")]
    fn finish_merge(&self, result: MergeVerticesOneRing) -> MergeVerticesOneRing {
        if self.probe_chain_integrity("merge_vertices_one_rings") {
            crate::state_history_push(self, "merge_vertices_one_rings");
        }
        result
    }

    #[cfg(not(feature = "instrumentation"))]
    fn finish_merge(&self, result: MergeVerticesOneRing) -> MergeVerticesOneRing {
        result
    }

    fn merge_remaining_unconnected(
        &mut self,
        mut one_ring_v_ids1: Vec<VertexId>,
        mut one_ring_v_ids2: Vec<VertexId>,
        result: &mut MergeVerticesOneRing,
    ) {
        while let Some(v_id) = one_ring_v_ids1.pop() {
            let Some(vert) = self.vertices.get(v_id) else {
                continue;
            };

            let mut vertices = vec![];

            for he_id in vert.boundary_halfedgdes(self) {
                let Some(he) = self.halfedges.get(he_id) else {
                    error!("Halfedge not found: {he_id:?}");
                    continue;
                };

                vertices.push(he.end_vertex);
            }

            if !vertices.is_empty() {
                one_ring_v_ids1.retain(|v_id| !vertices.contains(v_id));
                one_ring_v_ids2.retain(|v_id| !vertices.contains(v_id));

                let MergeVertices {
                    removed_vertices,
                    removed_halfedges,
                    removed_faces,
                } = self.merge_vertices(vertices);

                result.removed_vertices.extend(removed_vertices);
                result.removed_halfedges.extend(removed_halfedges);
                result.removed_faces.extend(removed_faces);
            }
        }
    }

    fn find_already_connected_pairings(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        shared_v_ids: &HashSet<VertexId>,
    ) -> Option<(Vec<FaceId>, HashSet<VertexId>, HashSet<HalfedgeId>)> {
        let mut connected_v_ids = HashSet::new();
        let mut connected_he_ids = HashSet::new();
        let mut already_connected_face_ids = vec![];

        // Only the first strip of already existing faces is taken. A second one
        // that isn't contiguous with it would mark its ring vertices as connected
        // too, and the ranges to connect are the complement of that: the region
        // between the two strips ends up in neither. Since the faces of both
        // strips are removed to make room for the new faces, that region is then
        // left as a hole. See `strip_found` below.
        let mut strip_found = false;

        for ((idx1, &v_id1), (idx2, &v_id2)) in one_ring_v_ids1
            .iter()
            .enumerate()
            .cartesian_product(one_ring_v_ids2.iter().enumerate())
        {
            if shared_v_ids.contains(&v_id1) || shared_v_ids.contains(&v_id2) {
                continue;
            }

            if v_id1 == v_id2 {
                connected_v_ids.insert(v_id1);
                continue;
            }

            if connected_v_ids.contains(&v_id1) && connected_v_ids.contains(&v_id2) {
                continue;
            }

            // The fan of the first strip is already extended over the whole strip
            // by `find_connected_triangle_fans` below, so any fan found from here
            // on belongs to a separate strip. Leave its faces in place and let the
            // new faces span it instead.
            if strip_found {
                continue;
            }

            if let Some(he_id) = self.halfedge_from_to(v_id1, v_id2) {
                let pairing = match self.find_triangle_fan(
                    he_id,
                    idx1,
                    idx2,
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &mut connected_he_ids,
                    &mut already_connected_face_ids,
                ) {
                    TriangleFan::Spanning(pairing) => pairing,
                    // The halfedge only touches the rings from the outside, so it
                    // doesn't connect them.
                    TriangleFan::NotSpanning => continue,
                    TriangleFan::Failed => return None,
                };

                #[cfg(feature = "rerun")]
                pairing.log_rerun(
                    "already_connected_init",
                    [one_ring_v_ids1, one_ring_v_ids2],
                    self,
                );

                let mut connected_pairings = self.find_connected_triangle_fans(
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &pairing,
                    -1,
                    &mut connected_he_ids,
                    &mut already_connected_face_ids,
                );

                connected_pairings.push(pairing.clone());

                connected_pairings.extend(self.find_connected_triangle_fans(
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &pairing,
                    1,
                    &mut connected_he_ids,
                    &mut already_connected_face_ids,
                ));

                for pairing in &connected_pairings {
                    for v_id in pairing.all_vertex_ids([one_ring_v_ids1, one_ring_v_ids2]) {
                        connected_v_ids.insert(v_id);
                    }
                }

                strip_found = true;
            }
        }

        Some((
            already_connected_face_ids,
            connected_v_ids,
            connected_he_ids,
        ))
    }

    fn find_connected_triangle_fans(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        pairing: &Pairing,
        idx_step: i32,
        connected_he_ids: &mut HashSet<HalfedgeId>,
        connected_face_ids: &mut Vec<FaceId>,
    ) -> Vec<Pairing> {
        let (single_v_ids, other_v_ids) = if pairing.single_range_idx == 0 {
            (one_ring_v_ids1, one_ring_v_ids2)
        } else {
            (one_ring_v_ids2, one_ring_v_ids1)
        };

        let next_single_idx = ((pairing.single_idx_in_range + single_v_ids.len()) as i32 + idx_step)
            as usize
            % single_v_ids.len();
        let next_v_id = single_v_ids[next_single_idx];

        let mut other_idx = if idx_step < 0 {
            *pairing.other_range.start()
        } else {
            *pairing.other_range.end()
        };
        other_idx %= other_v_ids.len();

        let current_single_v_id = single_v_ids[pairing.single_idx_in_range];

        let mut pairings = vec![];

        if self
            .face_with_vertices(next_v_id, current_single_v_id, other_v_ids[other_idx])
            .is_some()
        {
            // swap single range and other range
            let mut current_pairing = Pairing::new_triangle(
                1 - pairing.single_range_idx,
                other_idx,
                next_single_idx,
                pairing.single_idx_in_range,
                single_v_ids.len(),
            );

            self.extend_triangle_pairing_fan(
                one_ring_v_ids1,
                one_ring_v_ids2,
                &mut current_pairing,
                connected_he_ids,
                connected_face_ids,
            );

            #[cfg(feature = "rerun")]
            current_pairing.log_rerun(
                format!("extended/step_{}", idx_step),
                [one_ring_v_ids1, one_ring_v_ids2],
                self,
            );

            let next_pairings = self.find_connected_triangle_fans(
                one_ring_v_ids1,
                one_ring_v_ids2,
                &current_pairing,
                idx_step,
                connected_he_ids,
                connected_face_ids,
            );

            pairings.push(current_pairing);
            pairings.extend(next_pairings);
        }

        pairings
    }

    /// Finds the fan of already existing faces that connects the two one rings
    /// across the halfedge `he_id` (which runs from a ring 1 vertex to a ring 2
    /// vertex). See [`TriangleFan`] for the possible outcomes.
    #[allow(clippy::too_many_arguments)]
    fn find_triangle_fan(
        &self,
        he_id: HalfedgeId,
        idx1: usize,
        idx2: usize,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        connected_he_ids: &mut HashSet<HalfedgeId>,
        connected_face_ids: &mut Vec<FaceId>,
    ) -> TriangleFan {
        let he = unwrap_or_return!(
            self.halfedges.get(he_id),
            "Halfedge not found",
            TriangleFan::Failed
        );

        let mut current_pairing = Pairing {
            single_range_idx: 0,
            single_idx_in_range: idx1,
            other_range: idx2..=idx2,
        };

        // might not exist. we removed some faces (neighbors).
        let Some(opposite_v_id) = he.opposite_vertex(self) else {
            // The face on this side was one of the removed neighbor faces, so the
            // halfedge does span the region between the rings.
            return TriangleFan::Spanning(current_pairing);
        };

        if !self.create_triangle_pairing(
            one_ring_v_ids1,
            one_ring_v_ids2,
            idx1,
            idx2,
            opposite_v_id,
            &mut current_pairing,
        ) {
            let twin_id = unwrap_or_return!(he.twin, "Twin not found", TriangleFan::Failed);
            let twin = unwrap_or_return!(
                self.halfedges.get(twin_id),
                "Twin not found",
                TriangleFan::Failed
            );

            // might not exist. we removed some faces (neighbors).
            let Some(opposite_v_id) = twin.opposite_vertex(self) else {
                return TriangleFan::Spanning(current_pairing);
            };

            if !self.create_triangle_pairing(
                one_ring_v_ids1,
                one_ring_v_ids2,
                idx1,
                idx2,
                opposite_v_id,
                &mut current_pairing,
            ) {
                return TriangleFan::NotSpanning;
            }
        }

        self.extend_triangle_pairing_fan(
            one_ring_v_ids1,
            one_ring_v_ids2,
            &mut current_pairing,
            connected_he_ids,
            connected_face_ids,
        );

        TriangleFan::Spanning(current_pairing)
    }

    fn add_face_to_connected_he_ids(
        &self,
        face_id: FaceId,
        connected_he_ids: &mut HashSet<HalfedgeId>,
    ) {
        let face = unwrap_or_return!(
            self.faces.get(face_id),
            "failed to get start face for pairing fan"
        );
        connected_he_ids.extend(
            face.halfedges(self)
                .filter_map(|he_id| {
                    let he = self
                        .halfedges
                        .get(he_id)
                        .or_else(error_none!("Halfedge not found"))?;
                    Some([
                        he_id,
                        he.twin.or_else(error_none!("Twin halfedge not found"))?,
                    ])
                })
                .flatten(),
        );
    }

    fn extend_triangle_pairing_fan(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        current_pairing: &mut Pairing,
        connected_he_ids: &mut HashSet<HalfedgeId>,
        connected_face_ids: &mut Vec<FaceId>,
    ) {
        let (single_ids, other_ids) = if current_pairing.single_range_idx == 0 {
            (one_ring_v_ids1, one_ring_v_ids2)
        } else {
            (one_ring_v_ids2, one_ring_v_ids1)
        };

        let other_len = other_ids.len();
        let single_v_id = single_ids[current_pairing.single_idx_in_range % single_ids.len()];

        // Walk backwards from the start of other_range
        let mut range_start = *current_pairing.other_range.start();

        let start_face_id = unwrap_or_return!(
            self.face_with_vertices(
                single_v_id,
                other_ids[range_start % other_len],
                other_ids[*current_pairing.other_range.end() % other_len],
            ),
            "failed to find start face for pairing fan"
        );
        self.add_face_to_connected_he_ids(start_face_id, connected_he_ids);
        connected_face_ids.push(start_face_id);

        // Cap the number of steps so a complete face fan around the single vertex
        // can't make the walk wrap around the ring indefinitely.
        let mut steps = 0;
        loop {
            // Wrap backwards (using saturating to avoid underflow; handle wrap via modular arithmetic)
            let prev_idx = (range_start + other_len - 1) % other_len;
            let prev_v_id = other_ids[prev_idx];

            let boundary_v_id = other_ids[range_start % other_len];
            if let Some(face_id) = self.face_with_vertices(single_v_id, prev_v_id, boundary_v_id) {
                // Adjust range_start to go one step back; keep values un-modded so the
                // range arithmetic stays consistent with how ConnectPair::new handles wrapping.
                if range_start == 0 {
                    range_start = other_len - 1;
                } else {
                    range_start -= 1;
                }
                current_pairing.other_range = range_start..=*current_pairing.other_range.end();

                self.add_face_to_connected_he_ids(face_id, connected_he_ids);
                connected_face_ids.push(face_id);

                steps += 1;
                if steps >= other_len {
                    error!("Pairing fan wrapped around the entire one ring");
                    break;
                }
            } else {
                break;
            }
        }

        // Walk forwards from the end of other_range
        let mut range_end = *current_pairing.other_range.end();
        // Cap the number of steps so a complete face fan around the single vertex
        // can't make the walk wrap around the ring indefinitely.
        let mut steps = 0;
        loop {
            let next_idx = (range_end + 1) % other_len;
            let next_v_id = other_ids[next_idx];

            let boundary_v_id = other_ids[range_end % other_len];
            if let Some(face_id) = self.face_with_vertices(single_v_id, boundary_v_id, next_v_id) {
                range_end += 1;
                current_pairing.other_range = *current_pairing.other_range.start()..=range_end;

                self.add_face_to_connected_he_ids(face_id, connected_he_ids);
                connected_face_ids.push(face_id);

                steps += 1;
                if steps >= other_len {
                    error!("Pairing fan wrapped around the entire one ring");
                    break;
                }
            } else {
                break;
            }
        }
    }

    fn create_triangle_pairing(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        idx1: usize,
        idx2: usize,
        third_v_id: VertexId,
        current_pairing: &mut Pairing,
    ) -> bool {
        let (idx1, idx2) =
            if let Some(idx) = one_ring_v_ids1.iter().position(|v_id| *v_id == third_v_id) {
                current_pairing.single_range_idx = 1;
                current_pairing.single_idx_in_range = idx2;

                (idx, idx1)
            } else if let Some(idx) = one_ring_v_ids2.iter().position(|v_id| *v_id == third_v_id) {
                (idx, idx2)
            } else {
                return false;
            };

        let other_len = if current_pairing.single_range_idx == 0 {
            one_ring_v_ids2.len()
        } else {
            one_ring_v_ids1.len()
        };

        *current_pairing = Pairing::new_triangle(
            current_pairing.single_range_idx,
            current_pairing.single_idx_in_range,
            idx1,
            idx2,
            other_len,
        );

        true
    }

    fn cleanup_bookkeeping(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        marked_vertices: &mut HashSet<VertexId>,
        marked_halfedges: &mut HashSet<HalfedgeId>,
        result: &mut MergeVerticesOneRing,
    ) {
        for vertex_id in one_ring_v_ids1.iter().chain(one_ring_v_ids2).copied() {
            if !self.vertices.contains_key(vertex_id) {
                continue;
            }

            let cleanup = self.make_vertex_neighborhood_manifold(vertex_id);

            if !cleanup.added_vertices.is_empty() {
                marked_vertices.insert(vertex_id);
            }

            result.added_vertices.extend(cleanup.added_vertices.clone());
            marked_vertices.extend(cleanup.added_vertices);

            for removed_v_id in cleanup.removed_vertices {
                if result.added_vertices.contains(&removed_v_id) {
                    result.added_vertices.retain(|&v_id| v_id != removed_v_id);
                } else {
                    result.removed_vertices.push(removed_v_id);
                }

                marked_vertices.remove(&removed_v_id);
            }

            for removed_he_id in cleanup.removed_halfedges {
                if result.added_halfedges.contains(&removed_he_id) {
                    result
                        .added_halfedges
                        .retain(|&he_id| he_id != removed_he_id);
                } else {
                    result.removed_halfedges.push(removed_he_id);
                }

                marked_halfedges.remove(&removed_he_id);
            }

            for removed_face_id in cleanup.removed_faces {
                if result.added_faces.contains(&removed_face_id) {
                    result
                        .added_faces
                        .retain(|&face_id| face_id != removed_face_id);
                } else {
                    result.removed_faces.push(removed_face_id);
                }
            }
        }
    }

    fn add_planned_faces(
        &mut self,
        planned_faces: Vec<PlannedFace>,
        one_ring_set1: &HashSet<VertexId>,
        one_ring_set2: &HashSet<VertexId>,
        marked_halfedges: &mut HashSet<HalfedgeId>,
        result: &mut MergeVerticesOneRing,
    ) {
        let mut prev_he = None;

        for planned_face in planned_faces {
            let inserted = if let Some(prev_he) = prev_he {
                planned_face.add_to_mesh_graph_and_he(self, prev_he, result)
            } else {
                planned_face.add_to_mesh_graph(self, result)
            };

            let Some(inserted) = inserted else {
                // Depending on the topology inserting a face can fail sometimes.
                // This is fine: the insertion transaction rolled back whatever it
                // had created, so the mesh is exactly as it was before this face.
                // Record the skip - a systematically skipped face must not hide
                // as a successful merge.
                tracing::debug!("skipped planned face {planned_face:?}");
                #[cfg(test)]
                rim_probe::note_skipped_face();
                prev_he = None;
                continue;
            };

            prev_he = inserted.0;
            let inserted = inserted.1;

            for he_id in inserted.halfedge_ids.iter().copied() {
                // just inserted => must exist
                let he = self.halfedges[he_id];
                let start_v_id =
                    unwrap_or_return!(he.start_vertex(self), "Couldn't find start vertex");
                let end_v_id = he.end_vertex;

                if one_ring_set1.contains(&start_v_id) && one_ring_set2.contains(&end_v_id)
                    || one_ring_set2.contains(&start_v_id) && one_ring_set1.contains(&end_v_id)
                {
                    marked_halfedges.insert(he_id);
                }
            }

            result.added_halfedges.extend(inserted.halfedge_ids);
            result.added_faces.push(inserted.face_id);
        }
    }

    /// Fuse *parallel* edges left behind by the new faces.
    ///
    /// `add_or_get_boundary_edge` only ever reuses a free half in the direction
    /// it was asked for. When a planned face needs `a -> b` and that direction
    /// already carries a face, it adds a second, parallel edge between the same
    /// two vertices instead. That is fine while the region is still being
    /// rebuilt - a later face or the neighborhood cleanup usually claims the
    /// free halves - but when nothing does, the two halves stay face-less
    /// forever. The mesh then reports them as boundary (a "hole" with no face
    /// missing from it) and the two triangles that should share the edge each
    /// sit on their own copy of it.
    ///
    /// This is the sweep for that leftover: where two vertices are joined by
    /// more than one edge but exactly one face hangs off each direction, the two
    /// faced halves become each other's twin and the orphans are dropped. Any
    /// other shape is left alone - more than one face per direction is a
    /// genuinely non-manifold edge, which is the neighborhood cleanup's job.
    fn weld_parallel_edges(
        &mut self,
        region_v_ids: impl IntoIterator<Item = VertexId>,
        result: &mut MergeVerticesOneRing,
    ) {
        let mut seen = HashSet::<VertexId>::new();

        for v_id in region_v_ids {
            if !seen.insert(v_id) || !self.vertices.contains_key(v_id) {
                continue;
            }

            let Some(out_he_ids) = self.outgoing_halfedges.get(v_id).cloned() else {
                continue;
            };

            // End vertices this one reaches more than once, in a stable order.
            let mut ends = Vec::new();
            for he_id in &out_he_ids {
                if let Some(he) = self.halfedges.get(*he_id)
                    && !ends.contains(&he.end_vertex)
                {
                    ends.push(he.end_vertex);
                }
            }

            for other_v_id in ends {
                let forward = self.halfedges_from_to(v_id, other_v_id);
                let backward = self.halfedges_from_to(other_v_id, v_id);

                if forward.len() + backward.len() <= 2 {
                    continue;
                }

                let faced = |mesh: &Self, ids: &[HalfedgeId]| -> Vec<HalfedgeId> {
                    ids.iter()
                        .copied()
                        .filter(|he_id| {
                            mesh.halfedges
                                .get(*he_id)
                                .is_some_and(|he| he.face.is_some())
                        })
                        .collect()
                };

                let faced_forward = faced(self, &forward);
                let faced_backward = faced(self, &backward);

                if faced_forward.len() != 1 || faced_backward.len() != 1 {
                    continue;
                }

                let keep_forward = faced_forward[0];
                let keep_backward = faced_backward[0];

                self.halfedges[keep_forward].twin = Some(keep_backward);
                self.halfedges[keep_backward].twin = Some(keep_forward);

                for he_id in forward.into_iter().chain(backward) {
                    if he_id == keep_forward || he_id == keep_backward {
                        continue;
                    }
                    // Only the orphans go; anything with a face was counted above.
                    if self
                        .halfedges
                        .get(he_id)
                        .is_some_and(|he| he.face.is_none())
                    {
                        self.remove_only_halfedge(he_id);
                        result.removed_halfedges.push(he_id);
                    }
                }

                self.reseed_outgoing_if_dead(v_id);
                self.reseed_outgoing_if_dead(other_v_id);
            }
        }
    }

    /// Close triangular holes left in the merged region.
    ///
    /// The band planner can finish a full wrap around both rings and still leave
    /// one triangle of the closing seam unfilled - the last face and the first
    /// meet at a quad and only one of its two triangles gets planned (the start
    /// cap says `AlreadyConnected`, which is true of the edge but not of the
    /// face). Rather than another special case in the walk: a boundary loop of
    /// exactly three halfedges, all of whose vertices took part in this merge,
    /// has exactly one triangle that closes it, and closing it is always right
    /// for a region that was watertight before.
    ///
    /// Deliberately narrow. Only three-halfedge loops, only where every vertex
    /// is one this merge touched, and only where each of those vertices has a
    /// single boundary halfedge going out - so a genuine open border (the rim of
    /// a mesh that is meant to be open) is never sealed by accident.
    fn close_triangular_holes(
        &mut self,
        region_v_ids: &[VertexId],
        result: &mut MergeVerticesOneRing,
    ) {
        // The single outgoing boundary halfedge of a vertex, if it has exactly one.
        let sole_boundary_out = |mesh: &Self, v_id: VertexId| -> Option<HalfedgeId> {
            let out_hes = mesh.outgoing_halfedges.get(v_id)?;
            let mut found = None;
            for he_id in out_hes {
                if mesh
                    .halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.face.is_none())
                {
                    if found.is_some() {
                        return None;
                    }
                    found = Some(*he_id);
                }
            }
            found
        };

        for &start_v in region_v_ids {
            if !self.vertices.contains_key(start_v) {
                continue;
            }

            let Some(he1) = sole_boundary_out(self, start_v) else {
                continue;
            };
            let Some(v2) = self.halfedges.get(he1).map(|he| he.end_vertex) else {
                continue;
            };
            let Some(he2) = sole_boundary_out(self, v2) else {
                continue;
            };
            let Some(v3) = self.halfedges.get(he2).map(|he| he.end_vertex) else {
                continue;
            };
            let Some(he3) = sole_boundary_out(self, v3) else {
                continue;
            };
            let Some(v_back) = self.halfedges.get(he3).map(|he| he.end_vertex) else {
                continue;
            };

            // A closed three-halfedge loop, and nothing degenerate about it.
            if v_back != start_v || v2 == start_v || v3 == start_v || v2 == v3 {
                continue;
            }
            if !region_v_ids.contains(&v2) || !region_v_ids.contains(&v3) {
                continue;
            }

            if let Some(added) = self.add_face_from_halfedges(he1, he2) {
                result.added_faces.push(added.face_id);
                result.added_halfedges.extend(added.halfedge_ids);
            }
        }
    }

    /// Remove every face incident to `vertex_id`, walking the vertex's raw
    /// outgoing-halfedge list rather than [`Vertex::faces`].
    ///
    /// `faces()` follows one fan from the seed, so a pinched vertex (its star
    /// split into more than one fan) leaves the other fans' faces behind - and
    /// `H`, the boundary delta these removals produce, would then describe only
    /// part of the hole while the vertex stays attached. Used once per merge
    /// vertex, in order, so the rim each star opens can be attributed to its
    /// side.
    fn remove_vertex_faces(&mut self, vertex_id: VertexId, result: &mut MergeVerticesOneRing) {
        let mut face_ids: Vec<FaceId> = Vec::new();
        for he_id in self
            .outgoing_halfedges
            .get(vertex_id)
            .cloned()
            .unwrap_or_default()
        {
            if let Some(face_id) = self.halfedges.get(he_id).and_then(|he| he.face)
                && !face_ids.contains(&face_id)
            {
                face_ids.push(face_id);
            }
        }

        for face_id in face_ids {
            let (del_v, del_he) = self.remove_face(face_id);

            result.removed_faces.push(face_id);
            result.removed_vertices.extend(del_v);
            result.removed_halfedges.extend(del_he);
        }

        // The star must be gone in full: no live outgoing halfedge of the
        // vertex may still claim a face. A survivor means the enumeration
        // missed a fan and `H` is a partial hole.
        debug_assert!(
            self.outgoing_halfedges
                .get(vertex_id)
                .is_none_or(|out_hes| out_hes.iter().all(|he_id| self
                    .halfedges
                    .get(*he_id)
                    .is_none_or(|he| he.face.is_none()))),
            "star removal left faces on {vertex_id:?}"
        );
    }

    /// Dismantles the face (if any) that still claims `he_id` as a chain member.
    /// Planned bridge faces re-file halfedges that an overlapping earlier merge's
    /// bridge/fan may still own; re-filing a live chain member corrupts the old
    /// face's chain (face-stealing), so such faces are dismantled before the
    /// re-file. Only faces inside the merged region are affected: a halfedge that
    /// is re-filed by a planned face always has its other side owned by a face of
    /// the same region, never by an outside surface.
    fn dismantle_faced_member(&mut self, he_id: HalfedgeId, result: &mut MergeVerticesOneRing) {
        if let Some(he) = self.halfedges.get(he_id)
            && let Some(face_id) = he.face
            && self.faces.contains_key(face_id)
        {
            let (del_v, del_he) = self.remove_face(face_id);
            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(del_he);
            result.removed_vertices.extend(del_v);
        }
    }

    /// The `(start, end)` vertex pair of a live halfedge.
    fn halfedge_endpoints(&self, he_id: HalfedgeId) -> Option<(VertexId, VertexId)> {
        let he = self.halfedges.get(he_id)?;
        Some((he.start_vertex(self)?, he.end_vertex))
    }

    /// The live boundary half from `start` to `end`, if any.
    fn boundary_half_from_to(&self, start: VertexId, end: VertexId) -> Option<HalfedgeId> {
        self.halfedges_from_to(start, end)
            .into_iter()
            .find(|he_id| self.halfedges[*he_id].is_boundary())
    }

    /// Remove the edge pairs a failed planned-face insertion created. They are
    /// self-twinned and face-less by construction (nothing claims them before
    /// the face insertion runs), so removing both halves is safe.
    fn rollback_created_edges(&mut self, created: &mut Vec<HalfedgeId>) {
        for he_id in created.drain(..) {
            if self.halfedges.contains_key(he_id) {
                self.remove_only_halfedge_and_twin(he_id);
            }
        }
    }

    /// A pre-commit assertion: after a planned face was inserted, none of the
    /// edges created for it may be left face-less on both sides (wire). That
    /// can only happen if the insertion claimed a different half - a
    /// transaction bug.
    #[cfg(test)]
    fn debug_assert_created_edges_claimed(&self, created: &[HalfedgeId]) {
        for &he_id in created {
            let Some(he) = self.halfedges.get(he_id) else {
                continue;
            };
            let twin_faced = he
                .twin
                .and_then(|twin_id| self.halfedges.get(twin_id))
                .is_some_and(|twin| twin.face.is_some());
            debug_assert!(
                he.face.is_some() || twin_faced,
                "planned-face insertion left created edge {he_id:?} face-less on both sides"
            );
        }
    }

    /// A pre-commit assertion for the two inputs of `add_face_from_halfedges`:
    /// each must resolve to a boundary half with a twin and a resolvable start
    /// vertex. Otherwise the function's internal third-edge acquisition can
    /// create an edge and then fail on a missing twin, leaking an edge the
    /// caller's `created` bookkeeping never saw.
    #[cfg(test)]
    fn debug_assert_face_inputs_ready(&self, he_id1: HalfedgeId, he_id2: HalfedgeId) {
        for he_id in [he_id1, he_id2] {
            let resolved = self.boundary_he(he_id);
            debug_assert!(
                resolved.is_some(),
                "planned-face input {he_id:?} resolves to no boundary half"
            );
            if let Some(resolved) = resolved
                && let Some(he) = self.halfedges.get(resolved)
            {
                debug_assert!(
                    he.twin.is_some() && he.start_vertex(self).is_some(),
                    "planned-face input {resolved:?} is not insertion-ready (missing twin or start vertex)"
                );
            }
        }
    }

    #[instrument(skip_all)]
    fn flip_and_collapse_single_shared_edge_if_below_threshold(
        &mut self,
        single_shared_he_id: HalfedgeId,
        flip_threshold_sqr: f32,
        result: &mut MergeVerticesOneRing,
    ) -> bool {
        // checked already in the call chain
        let he = self.halfedges[single_shared_he_id];

        let twin_id = unwrap_or_return!(he.twin, "Twin not found", false);
        let twin = unwrap_or_return!(self.halfedges.get(twin_id), "Twin not found", false);

        let other_v_id1 =
            unwrap_or_return!(he.opposite_vertex(self), "Opposite vertex not found", false);
        let other_v_id2 = unwrap_or_return!(
            twin.opposite_vertex(self),
            "Opposite vertex not found",
            false
        );

        let pos1 = *unwrap_or_return!(self.positions.get(other_v_id1), "Position not found", false);
        let pos2 = *unwrap_or_return!(self.positions.get(other_v_id2), "Position not found", false);

        if pos1.distance_squared(pos2) <= flip_threshold_sqr {
            tracing::debug!("Flipping edge {single_shared_he_id:?}");

            self.flip_edge(single_shared_he_id);

            let CollapseEdge {
                removed_vertices,
                removed_halfedges,
                removed_faces,
                added_vertices,
            } = self.collapse_edge(single_shared_he_id);

            result.added_vertices.extend(added_vertices);
            result.removed_vertices.extend(removed_vertices);
            result.removed_halfedges.extend(removed_halfedges);
            result.removed_faces.extend(removed_faces);

            true
        } else {
            false
        }
    }

    fn plan_new_faces(
        &self,
        range_pairs_to_connect: &[ConnectPair],
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
    ) -> Vec<PlannedFace> {
        let mut planned_faces = Vec::with_capacity(one_ring_v_ids1.len());

        for range_pair_to_connect in range_pairs_to_connect {
            let start_pairing_index = planned_faces.len();

            let pairings = range_pair_to_connect.compute_pairings();

            tracing::debug!("Pairings: {:#?}", pairings);

            if pairings.is_empty() {
                continue;
            }

            let mut prev_single_v_id = None;
            let mut prev_other_v_id = None;

            if matches!(range_pair_to_connect.start_cap, ConnectPairCap::Open) {
                let last_pairing = pairings.last().unwrap();

                let (s, o) = last_pairing.last_pair([one_ring_v_ids1, one_ring_v_ids2]);

                prev_single_v_id = Some(s);
                prev_other_v_id = Some(o);
            }

            for pairing in pairings {
                if let Some(prev_single_v_id) = prev_single_v_id
                    && let Some(prev_other_v_id) = prev_other_v_id
                {
                    // add the quad between the previous and current pairings
                    let (single_v_id, other_v_id) =
                        pairing.first_pair([one_ring_v_ids1, one_ring_v_ids2]);

                    let face_order = if prev_single_v_id == prev_other_v_id {
                        PlannedFaceOrder::Start
                    } else if single_v_id == other_v_id {
                        PlannedFaceOrder::End
                    } else {
                        PlannedFaceOrder::Middle
                    };

                    // make sure the triangle vertices are CCW
                    if pairing.single_range_idx == 0 {
                        if prev_single_v_id != prev_other_v_id {
                            planned_faces.push(PlannedFace::new(
                                prev_single_v_id,
                                single_v_id,
                                prev_other_v_id,
                                face_order,
                            ));
                        }
                        if single_v_id != other_v_id {
                            planned_faces.push(PlannedFace::new(
                                prev_other_v_id,
                                single_v_id,
                                other_v_id,
                                face_order,
                            ));
                        }
                    } else {
                        if prev_single_v_id != prev_other_v_id {
                            planned_faces.push(PlannedFace::new(
                                prev_single_v_id,
                                prev_other_v_id,
                                single_v_id,
                                face_order,
                            ));
                        }
                        if single_v_id != other_v_id {
                            planned_faces.push(PlannedFace::new(
                                prev_other_v_id,
                                other_v_id,
                                single_v_id,
                                face_order,
                            ));
                        }
                    }
                }

                // add the faces of the current pairing
                planned_faces.extend(pairing.fill_faces([one_ring_v_ids1, one_ring_v_ids2]));

                let (single_v_id, other_v_id) =
                    pairing.last_pair([one_ring_v_ids1, one_ring_v_ids2]);

                prev_single_v_id = Some(single_v_id);
                prev_other_v_id = Some(other_v_id);
            }

            if planned_faces.len() == start_pairing_index {
                // This range pair produced no faces (e.g. a single shared vertex).
                continue;
            }

            if !matches!(
                range_pair_to_connect.start_cap,
                ConnectPairCap::AlreadyConnected
            ) {
                planned_faces[start_pairing_index].order = PlannedFaceOrder::Start;
            }

            let len = planned_faces.len();
            planned_faces[len - 1].order = if len - 1 == start_pairing_index {
                PlannedFaceOrder::Single
            } else {
                PlannedFaceOrder::End
            };
        }

        planned_faces
    }

    #[instrument(skip_all)]
    fn check_and_flip_single_shared_he(
        &mut self,
        shared_he_ids: &HashSet<HalfedgeId>,
        flip_threshold_sqr: f32,
        result: &mut MergeVerticesOneRing,
    ) -> bool {
        if shared_he_ids.len() == 1 {
            let shared_he_id = shared_he_ids.iter().copied().next().unwrap();

            #[cfg(feature = "rerun")]
            self.log_he_rerun("common_one_ring_he", shared_he_id);

            return self.flip_and_collapse_single_shared_edge_if_below_threshold(
                shared_he_id,
                flip_threshold_sqr,
                result,
            );
        }

        false
    }

    #[instrument(skip_all)]
    #[allow(clippy::too_many_arguments)]
    fn compute_range_pairs_to_connect(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        one_ring_he_ids1: &[HalfedgeId],
        one_ring_he_ids2: &[HalfedgeId],
        shared_v_ids: &HashSet<VertexId>,
        shared_he_ids: &HashSet<HalfedgeId>,
        mut connected_v_ids: HashSet<VertexId>,
        mut connected_he_ids: HashSet<HalfedgeId>,
    ) -> Vec<ConnectPair> {
        let mut range_pairs_to_connect = vec![];

        let (orig_start_idx1, orig_start_idx2, orig_start_cap) = unwrap_or_return!(
            self.find_start_indices(
                one_ring_v_ids1,
                one_ring_v_ids2,
                one_ring_he_ids1,
                one_ring_he_ids2,
                shared_v_ids,
                shared_he_ids,
                &connected_v_ids,
                &connected_he_ids,
            ),
            "Couldn't find start indices",
            range_pairs_to_connect
        );

        tracing::debug!(
            "start idx1: {}, start idx2: {}",
            orig_start_idx1,
            orig_start_idx2
        );

        let len1 = one_ring_v_ids1.len();
        let len2 = one_ring_v_ids2.len();

        let mut start_idx1 = orig_start_idx1;
        let mut start_idx2 = orig_start_idx2;
        let mut start_cap = orig_start_cap;

        let mut end_idx1 = (start_idx1 + 1) % len1;
        let mut end_idx2 = (start_idx2 + 1) % len2;

        #[cfg(feature = "rerun")]
        {
            self.log_verts_w_labels_rerun(
                "pairs_start_idx",
                &[one_ring_v_ids1[start_idx1], one_ring_v_ids2[start_idx2]],
                &["1", "2"],
            );
            self.log_vert_rerun("pairs_end_idx1", one_ring_v_ids1[end_idx1]);
            self.log_vert_rerun("pairs_end_idx2", one_ring_v_ids2[end_idx2]);
        }
        let mut v_id1;
        let mut v_id2;

        // Set when the walk comes back around to where it started: the ranges
        // then already cover the whole ring and the leftover push below would
        // duplicate the first one.
        let mut wrapped = false;

        while end_idx1 != orig_start_idx1 {
            v_id1 = one_ring_v_ids1[end_idx1];
            v_id2 = one_ring_v_ids2[end_idx2];

            if !self.vertices.contains_key(v_id1) || !self.vertices.contains_key(v_id2) {
                break;
            }

            let mut end_cap = ConnectPairCap::Open;

            if shared_v_ids.contains(&v_id1) {
                let start_end_idx2 = end_idx2;
                while v_id2 != v_id1 {
                    end_idx2 = (end_idx2 + 1) % len2;
                    v_id2 = one_ring_v_ids2[end_idx2];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx2", v_id2);

                    if end_idx2 == start_end_idx2 {
                        error!("Full cycle without finding shared vertex in one ring 2");
                        return range_pairs_to_connect;
                    }
                }

                end_cap = ConnectPairCap::Closed;
            } else if shared_v_ids.contains(&v_id2) {
                let start_end_idx1 = end_idx1;
                while v_id1 != v_id2 {
                    end_idx1 = (end_idx1 + 1) % len1;
                    v_id1 = one_ring_v_ids1[end_idx1];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx1", v_id1);

                    if end_idx1 == start_end_idx1 {
                        error!("Full cycle without finding shared vertex in one ring 1");
                        return range_pairs_to_connect;
                    }
                }

                end_cap = ConnectPairCap::Closed;
            } else if connected_v_ids.contains(&v_id1) {
                let start_end_idx2 = end_idx2;
                while !connected_v_ids.contains(&v_id2) {
                    end_idx2 = (end_idx2 + 1) % len2;
                    v_id2 = one_ring_v_ids2[end_idx2];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx2", v_id2);

                    if end_idx2 == start_end_idx2 {
                        error!("Full cycle without finding connected vertex in one ring 2");
                        return range_pairs_to_connect;
                    }
                }

                end_cap = ConnectPairCap::AlreadyConnected;
            } else if connected_v_ids.contains(&v_id2) {
                let start_end_idx1 = end_idx1;
                while !connected_v_ids.contains(&v_id1) {
                    end_idx1 = (end_idx1 + 1) % len1;
                    v_id1 = one_ring_v_ids1[end_idx1];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx1", v_id1);

                    if end_idx1 == start_end_idx1 {
                        error!("Full cycle without finding connected vertex in one ring 1");
                        return range_pairs_to_connect;
                    }
                }

                end_cap = ConnectPairCap::AlreadyConnected;
            }

            if !matches!(end_cap, ConnectPairCap::Open) {
                range_pairs_to_connect.push(ConnectPair::new(
                    start_idx1..=end_idx1,
                    len1,
                    start_idx2..=end_idx2,
                    len2,
                    start_cap,
                ));

                self.remember_range_pair_connections(
                    start_idx1,
                    end_idx1,
                    start_idx2,
                    end_idx2,
                    len1,
                    len2,
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    one_ring_he_ids1,
                    one_ring_he_ids2,
                    &mut connected_v_ids,
                    &mut connected_he_ids,
                );

                if let Some((idx1, idx2, start)) = self.find_start_indices(
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    one_ring_he_ids1,
                    one_ring_he_ids2,
                    shared_v_ids,
                    shared_he_ids,
                    &connected_v_ids,
                    &connected_he_ids,
                ) {
                    start_idx1 = idx1;
                    start_idx2 = idx2;
                    start_cap = start;
                } else {
                    return range_pairs_to_connect;
                }

                end_idx1 = (start_idx1 + 1) % len1;
                end_idx2 = (start_idx2 + 1) % len2;

                #[cfg(feature = "rerun")]
                {
                    self.log_verts_w_labels_rerun(
                        "pairs_start_idx",
                        &[one_ring_v_ids1[start_idx1], one_ring_v_ids2[start_idx2]],
                        &["1", "2"],
                    );
                    self.log_vert_rerun("pairs_end_idx1", one_ring_v_ids1[end_idx1]);
                    self.log_vert_rerun("pairs_end_idx2", one_ring_v_ids2[end_idx2]);
                }

                if start_idx1 == orig_start_idx1 {
                    wrapped = true;
                    break;
                }
            } else {
                end_idx1 = (end_idx1 + 1) % len1;
                end_idx2 = (end_idx2 + 1) % len2;

                #[cfg(feature = "rerun")]
                {
                    self.log_vert_rerun("pairs_end_idx1", one_ring_v_ids1[end_idx1]);
                    self.log_vert_rerun("pairs_end_idx2", one_ring_v_ids2[end_idx2]);
                }
            }
        }

        // The walk advances both rings in lockstep and only lets one catch up
        // when the loop body sees a shared vertex - but it stops as soon as ring
        // 1 is back where it started, so the final cap may never get that
        // treatment. Close the gap here the same way the loop body would, or the
        // leftover range below covers one step of the other ring's arc instead
        // of all of it, and the faces over the rest are never planned (a lens
        // whose two shared vertices are adjacent on one ring and far apart on
        // the other: the merge left an open quad).
        let end_v_id1 = one_ring_v_ids1[end_idx1];
        if shared_v_ids.contains(&end_v_id1) {
            let from = end_idx2;
            while one_ring_v_ids2[end_idx2] != end_v_id1 {
                end_idx2 = (end_idx2 + 1) % len2;
                if end_idx2 == from {
                    break;
                }
            }
        }
        let end_v_id2 = one_ring_v_ids2[end_idx2];
        if shared_v_ids.contains(&end_v_id2) {
            let from = end_idx1;
            while one_ring_v_ids1[end_idx1] != end_v_id2 {
                end_idx1 = (end_idx1 + 1) % len1;
                if end_idx1 == from {
                    break;
                }
            }
        }

        // How far each range actually reaches, walking forward with wraparound -
        // the direction `ConnectPair` reads its ranges in. A symmetric distance
        // would call the arc `4..=3` of a 6-ring one step long when it spans
        // five.
        let span1 = (end_idx1 + len1 - start_idx1) % len1;
        let span2 = (end_idx2 + len2 - start_idx2) % len2;

        if range_pairs_to_connect.is_empty() {
            let (end1, end2, cap) = if shared_v_ids.is_empty() {
                (
                    (orig_start_idx1 + len1 - 1).rem_euclid(len1),
                    (orig_start_idx2 + len2 - 1).rem_euclid(len2),
                    ConnectPairCap::Open,
                )
            } else {
                (orig_start_idx1, orig_start_idx2, ConnectPairCap::Closed)
            };

            range_pairs_to_connect.push(ConnectPair::new(
                orig_start_idx1..=end1,
                len1,
                orig_start_idx2..=end2,
                len2,
                cap,
            ));
        } else if !wrapped && (span1 > 0 || span2 > 0) {
            // Whatever is left between the last emitted pair and the start of the
            // walk still has to be spanned. This used to require a span of two or
            // more on one side, which silently dropped the final segment of a
            // *lens* merge - two one-rings sharing two vertices, where the walk
            // emits the long segment and the short one is left as a single step
            // on both rings. Its faces had already been torn out with the merged
            // vertices' stars, so nothing re-filled them and the merge ended with
            // a hole (the intermittent weld defect: the close left boundary
            // halfedges behind).
            range_pairs_to_connect.push(ConnectPair::new(
                start_idx1..=end_idx1,
                len1,
                start_idx2..=end_idx2,
                len2,
                start_cap,
            ));
        }

        #[cfg(feature = "rerun")]
        {
            for range_pair in &range_pairs_to_connect {
                let mut v1s = vec![];
                let mut v2s = vec![];
                let mut l1s = vec![];
                let mut l2s = vec![];

                for i in range_pair.ranges[0].clone() {
                    let v_id = one_ring_v_ids1[i % len1];
                    v1s.push(v_id);
                    l1s.push(format!("{i} - {v_id:?}"));
                }
                for i in range_pair.ranges[1].clone() {
                    let v_id = one_ring_v_ids2[i % len2];
                    v2s.push(v_id);
                    l2s.push(format!("{i} - {v_id:?}"));
                }

                self.log_verts_w_labels_rerun("range_pair_1", &v1s, &l1s);
                self.log_verts_w_labels_rerun("range_pair_2", &v2s, &l2s);
            }
        }

        range_pairs_to_connect
    }

    #[instrument(skip(self))]
    #[allow(clippy::too_many_arguments)]
    fn remember_range_pair_connections(
        &mut self,
        start_idx1: usize,
        end_idx1: usize,
        start_idx2: usize,
        end_idx2: usize,
        len1: usize,
        len2: usize,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        one_ring_he_ids1: &[HalfedgeId],
        one_ring_he_ids2: &[HalfedgeId],
        connected_v_ids: &mut HashSet<VertexId>,
        connected_he_ids: &mut HashSet<HalfedgeId>,
    ) {
        let mut idx = start_idx1;
        loop {
            connected_v_ids.insert(one_ring_v_ids1[idx]);
            if idx == end_idx1 {
                break;
            }
            let he_id = one_ring_he_ids1[idx];
            connected_he_ids.insert(he_id);

            // could have been deleted by neighbor face deletion
            if let Some(he) = self.halfedges.get(he_id) {
                // halfedge existence already checked in `one_ring()`
                if let Some(twin_id) = he.twin {
                    connected_he_ids.insert(twin_id);
                } else {
                    error!("halfedge {:?} has no twin", he_id);
                }
            }

            idx = (idx + 1) % len1;
        }

        idx = start_idx2;
        loop {
            connected_v_ids.insert(one_ring_v_ids2[idx]);
            if idx == end_idx2 {
                break;
            }
            let he_id = one_ring_he_ids2[idx];
            connected_he_ids.insert(he_id);

            // could have been deleted by neighbor face deletion
            if let Some(he) = self.halfedges.get(he_id) {
                // halfedge existence already checked in `one_ring()`
                if let Some(twin_id) = he.twin {
                    connected_he_ids.insert(twin_id);
                } else {
                    error!("halfedge {:?} has no twin", he_id);
                }
            }

            idx = (idx + 1) % len2;
        }
    }

    #[instrument(skip(self))]
    #[allow(clippy::too_many_arguments)]
    fn find_shared_start_indices_from_ring(
        &self,
        one_ring_v_ids: &[VertexId],
        other_one_ring_v_ids: &[VertexId],
        one_ring_he_ids: &[HalfedgeId],
        other_one_ring_he_ids: &[HalfedgeId],
        shared_v_ids: &HashSet<VertexId>,
        shared_he_ids: &HashSet<HalfedgeId>,
        connected_v_ids: &HashSet<VertexId>,
        connected_he_ids: &HashSet<HalfedgeId>,
    ) -> Option<(usize, usize, ConnectPairCap)> {
        let len = one_ring_v_ids.len();
        let other_len = other_one_ring_v_ids.len();

        debug_assert_eq!(len, one_ring_he_ids.len());
        debug_assert_eq!(other_len, other_one_ring_he_ids.len());

        for (he_idx, &he_id1) in one_ring_he_ids.iter().enumerate() {
            // A *dead* ring halfedge is not a reason to skip the position: it
            // means `remove_neighbor_faces` consumed that edge with the merged
            // vertices' stars, which is precisely a position that still needs
            // connecting. Requiring it to be alive made the scan give up on
            // exactly the ring stretches the band has to span, and the op then
            // returned "nothing to connect" having already torn both stars out.
            if !shared_he_ids.contains(&he_id1) && !connected_he_ids.contains(&he_id1) {
                // walk backwards until we find a connection between the two one-rings

                let mut start_idx = he_idx;
                let mut start_he_id = &one_ring_he_ids[start_idx];

                let mut start_v_id = &one_ring_v_ids[start_idx];

                while self.halfedges.contains_key(*start_he_id)
                    && !shared_he_ids.contains(start_he_id)
                    && !connected_he_ids.contains(start_he_id)
                    && self.vertices.contains_key(*start_v_id)
                    && !shared_v_ids.contains(start_v_id)
                    && !connected_v_ids.contains(start_v_id)
                {
                    start_idx = (start_idx + len - 1) % len;
                    start_he_id = &one_ring_he_ids[start_idx];
                    start_v_id = &one_ring_v_ids[start_idx];

                    if start_idx == he_idx {
                        return None;
                    }
                }

                // find the corresponding vertex in the other one-ring

                // either it's the same vertex ...
                if let Some(other_idx) = other_one_ring_v_ids
                    .iter()
                    .position(|v_id| start_v_id == v_id)
                {
                    return Some((start_idx, other_idx, ConnectPairCap::Closed));
                }

                // ... or it's a different vertex that is connected by a halfedge
                for (other_idx, &other_v_id) in other_one_ring_v_ids.iter().enumerate() {
                    if self.halfedge_from_to(*start_v_id, other_v_id).is_some() {
                        let other_he_id = &other_one_ring_he_ids[other_idx];

                        if self.halfedges.contains_key(*other_he_id)
                            && !shared_he_ids.contains(other_he_id)
                            && !connected_he_ids.contains(other_he_id)
                        {
                            return Some((start_idx, other_idx, ConnectPairCap::AlreadyConnected));
                        }
                    }
                }
            }
        }

        None
    }

    #[allow(clippy::too_many_arguments)]
    #[instrument(skip(self))]
    fn find_start_indices(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        one_ring_he_ids1: &[HalfedgeId],
        one_ring_he_ids2: &[HalfedgeId],
        shared_v_ids: &HashSet<VertexId>,
        shared_he_ids: &HashSet<HalfedgeId>,
        connected_v_ids: &HashSet<VertexId>,
        connected_he_ids: &HashSet<HalfedgeId>,
    ) -> Option<(usize, usize, ConnectPairCap)> {
        if !shared_v_ids.is_empty() || !connected_v_ids.is_empty() {
            self.find_shared_start_indices_from_ring(
                one_ring_v_ids1,
                one_ring_v_ids2,
                one_ring_he_ids1,
                one_ring_he_ids2,
                shared_v_ids,
                shared_he_ids,
                connected_v_ids,
                connected_he_ids,
            )
        } else {
            let first_v_id = one_ring_v_ids1[0];
            let first_pos = *self
                .positions
                .get(first_v_id)
                .or_else(error_none!("Position not found"))?;

            let mut min_dist_sqr = f32::INFINITY;
            let mut start_idx2 = 0;

            for (idx, v_id) in one_ring_v_ids2.iter().enumerate() {
                let pos = self
                    .positions
                    .get(*v_id)
                    .or_else(error_none!("Position not found"))?;

                let dist_sqr = pos.distance_squared(first_pos);

                if dist_sqr < min_dist_sqr {
                    min_dist_sqr = dist_sqr;
                    start_idx2 = idx;
                }
            }

            Some((0, start_idx2, ConnectPairCap::Open))
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConnectPairCap {
    /// Indices reference the same vertex, thus forming a closed polygon.
    Closed,
    /// Indices reference different vertices, thus forming an open polygon.
    Open,
    /// Indices are already connected by neighboring faces, so no need to connect them again.
    AlreadyConnected,
}

/// Two ranges of elements that need to be connected by faces.
#[derive(Debug)]
struct ConnectPair {
    /// Wether the first indices in the two ranges reference the same vertex, thus forming a closed polygon.
    start_cap: ConnectPairCap,

    /// The pair of ranges of vertex indices that need to be connected.
    ranges: [RangeInclusive<usize>; 2],
}

impl ConnectPair {
    fn new(
        mut range1: RangeInclusive<usize>,
        len1: usize,
        mut range2: RangeInclusive<usize>,
        len2: usize,
        start_cap: ConnectPairCap,
    ) -> Self {
        if range1.end() <= range1.start() {
            range1 = (*range1.start())..=(*range1.end() + len1);
        }
        if range2.end() <= range2.start() {
            range2 = (*range2.start())..=(*range2.end() + len2);
        }

        ConnectPair {
            start_cap,
            ranges: [range1, range2],
        }
    }

    // This is a modified Bresenham algorithm usually used for drawing lines on a grid.
    fn compute_pairings(&self) -> Vec<Pairing> {
        let range1 = self.ranges[0].clone();
        let range2 = self.ranges[1].clone();

        let count1 = range1.clone().count() as i32;
        let count2 = range2.clone().count() as i32;

        let (single_range_idx, single_count, other_count, single_range, other_range) =
            if count1 > count2 {
                (1, count2, count1, range2, range1)
            } else {
                (0, count1, count2, range1, range2)
            };

        let mut single_idx_in_range = *single_range.start();

        if single_count == 0 {
            return vec![];
        }
        if single_count == 1 {
            return vec![Pairing {
                single_range_idx,
                single_idx_in_range,
                other_range,
            }];
        }

        let mut error = 2 * single_count - other_count;

        let mut other_start = *other_range.start();
        let mut other_end = other_start;

        let mut pairings = Vec::new();

        while other_end <= *other_range.end() {
            if error > 0 {
                pairings.push(Pairing {
                    single_range_idx,
                    single_idx_in_range,
                    other_range: other_start..=other_end,
                });

                other_start = other_end + 1;
                single_idx_in_range += 1;
                error += 2 * (single_count - other_count);
            } else {
                error += 2 * single_count;
            }

            other_end += 1;
        }

        if let Some(last_pairing) = pairings.last_mut() {
            last_pairing.other_range = *last_pairing.other_range.start()..=*other_range.end();
        }

        pairings
    }
}

/// The outcome of [`MeshGraph::find_triangle_fan`].
enum TriangleFan {
    /// The halfedge spans the region between the two one rings. Holds the pairing
    /// of the fan of already existing faces around it.
    Spanning(Pairing),

    /// The halfedge doesn't span the region between the two one rings: both of its
    /// faces are intact and neither of them has its third vertex on one of the
    /// rings, so the halfedge runs along the outside of the rings instead of
    /// across the region between them. The vertices it joins must not count as
    /// connected.
    NotSpanning,

    /// The mesh connectivity around the halfedge is broken. The merge is aborted.
    Failed,
}

/// Pairs one element from one range with one or more elements from the other range.
#[derive(Debug, Clone)]
struct Pairing {
    /// The index of the range in which each single element is paired to one or more elements in the other range.
    /// Can only be `0` or `1`.
    single_range_idx: usize,

    /// The index of the single element in the range referenced by `single_range_idx`.
    single_idx_in_range: usize,

    /// The range of elements in the other range that are paired to the single element referenced in `single_idx_in_range`.
    other_range: RangeInclusive<usize>,
}

impl Pairing {
    fn new_triangle(
        single_range_idx: usize,
        single_idx_in_range: usize,
        other_idx1: usize,
        other_idx2: usize,
        other_len: usize,
    ) -> Self {
        let other_range = if other_idx1 == other_idx2 {
            other_idx1..=other_idx2
        } else if (other_idx1 as i32 - other_idx2 as i32).abs() == 1 {
            other_idx1.min(other_idx2)..=other_idx1.max(other_idx2)
        } else {
            other_idx1.max(other_idx2)..=other_idx1.min(other_idx2) + other_len
        };

        Self {
            single_range_idx,
            single_idx_in_range,
            other_range,
        }
    }

    fn all_vertex_ids(&self, v_ids: [&[VertexId]; 2]) -> Vec<VertexId> {
        let single_ids = &v_ids[self.single_range_idx];
        let other_ids = &v_ids[1 - self.single_range_idx];

        let mut all_ids = vec![];

        all_ids.push(single_ids[self.single_idx_in_range % single_ids.len()]);

        let mut idx = *self.other_range.start() % other_ids.len();
        all_ids.push(other_ids[idx]);
        while idx != *self.other_range.end() % other_ids.len() {
            idx += 1;
            idx %= other_ids.len();
            all_ids.push(other_ids[idx]);
        }

        all_ids
    }

    fn first_pair(&self, v_ids: [&[VertexId]; 2]) -> (VertexId, VertexId) {
        let single_ids = &v_ids[self.single_range_idx];
        let other_ids = &v_ids[1 - self.single_range_idx];

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let other_v_id = other_ids[*self.other_range.start() % other_ids.len()];

        (single_v_id, other_v_id)
    }

    fn last_pair(&self, v_ids: [&[VertexId]; 2]) -> (VertexId, VertexId) {
        let single_ids = &v_ids[self.single_range_idx];
        let other_ids = &v_ids[1 - self.single_range_idx];

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let other_v_id = other_ids[*self.other_range.end() % other_ids.len()];

        (single_v_id, other_v_id)
    }

    fn fill_faces(&self, v_ids: [&[VertexId]; 2]) -> Vec<PlannedFace> {
        let single_ids = v_ids[self.single_range_idx];
        let other_ids = v_ids[1 - self.single_range_idx];

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let mut faces = Vec::<PlannedFace>::new();

        let mut others = self.other_range.clone();

        if others.start() == others.end() {
            return faces;
        }

        let mut prev_other_v_id = other_ids[others.next().unwrap() % other_ids.len()];
        let mut face_order = PlannedFaceOrder::Middle;

        if prev_other_v_id == single_v_id {
            face_order = PlannedFaceOrder::Start;

            if let Some(second_idx) = others.next() {
                prev_other_v_id = other_ids[second_idx % other_ids.len()];
            } else {
                return faces;
            }
        }

        for other_idx in others {
            let other_v_id = other_ids[other_idx % other_ids.len()];

            if other_v_id == single_v_id {
                break;
            }

            if other_v_id == prev_other_v_id {
                // Consecutive duplicates in the "other" ring would produce a
                // degenerate (V, S, V) self-connected face. Skip the face but
                // keep advancing so the following triangles stay well-formed.
                prev_other_v_id = other_v_id;
                continue;
            }

            // make sure the triangle vertices are CCW
            if self.single_range_idx == 0 {
                faces.push(PlannedFace::new(
                    prev_other_v_id,
                    single_v_id,
                    other_v_id,
                    face_order,
                ));
            } else {
                faces.push(PlannedFace::new(
                    prev_other_v_id,
                    other_v_id,
                    single_v_id,
                    face_order,
                ));
            }

            face_order = PlannedFaceOrder::Middle;
            prev_other_v_id = other_v_id;
        }

        faces
    }

    #[cfg(feature = "rerun")]
    fn log_rerun(
        &self,
        label: impl AsRef<str>,
        v_ids: [&[VertexId]; 2],
        mesh_graph: &MeshGraph,
    ) -> Option<()> {
        use crate::utils::vec3_array;

        let single_v_ids = v_ids[self.single_range_idx];
        let other_v_ids = v_ids[1 - self.single_range_idx];

        let single_v_id = single_v_ids[self.single_idx_in_range];
        let single_pos = vec3_array(mesh_graph.positions.get(single_v_id)?);

        let mut other_pos = vec![];
        let mut idx = self.other_range.start() % other_v_ids.len();

        other_pos.push(vec3_array(mesh_graph.positions.get(other_v_ids[idx])?));
        while idx != *self.other_range.end() % other_v_ids.len() {
            idx += 1;
            idx %= other_v_ids.len();
            other_pos.push(vec3_array(mesh_graph.positions.get(other_v_ids[idx])?));
        }

        crate::RR
            .log(
                format!("pairing/{}/single", label.as_ref()),
                &rerun::Points3D::new([single_pos]),
            )
            .unwrap();
        crate::RR
            .log(
                format!("pairing/{}/other", label.as_ref()),
                &rerun::LineStrips3D::new(
                    other_pos
                        .into_iter()
                        .map(|other_pos| [single_pos, other_pos]),
                ),
            )
            .unwrap();

        Some(())
    }
}

/// Represents a triangle that is planned to be added to the mesh graph.
///
/// v1 ()◀───────────────() new_he_v2
///      ╲              ▲
///       ╲            ╱
///        ╲          ╱
///         ╲        ╱
///          ╲      ╱
///           ╲    ╱
///            ▼  ╱
///             ()
///         new_he_v1
///
#[derive(Debug)]
struct PlannedFace {
    order: PlannedFaceOrder,
    v1: VertexId,
    new_he_v1: VertexId,
    new_he_v2: VertexId,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum PlannedFaceOrder {
    Start,
    Middle,
    End,
    Single,
}

impl PlannedFace {
    fn new(
        v_id1: VertexId,
        new_he_v_id1: VertexId,
        new_he_v_id2: VertexId,
        order: PlannedFaceOrder,
    ) -> Self {
        PlannedFace {
            order,
            v1: v_id1,
            new_he_v1: new_he_v_id1,
            new_he_v2: new_he_v_id2,
        }
    }

    /// Pre-insertion validation, before any mutation: the three vertices must
    /// be distinct and alive. A failing face is skipped; a transaction never
    /// starts.
    fn validate_planned_face(&self, mesh_graph: &MeshGraph) -> Option<()> {
        let v_ids = [self.v1, self.new_he_v1, self.new_he_v2];
        if v_ids[0] == v_ids[1] || v_ids[1] == v_ids[2] || v_ids[0] == v_ids[2] {
            error!("Skipping degenerate planned face with repeated vertices: {v_ids:?}");
            return None;
        }
        for v_id in v_ids {
            if !mesh_graph.vertices.contains_key(v_id) {
                error!("Skipping planned face with dead vertex {v_id:?}: {v_ids:?}");
                return None;
            }
        }
        Some(())
    }

    #[instrument(skip(mesh_graph, result))]
    fn add_to_mesh_graph(
        &self,
        mesh_graph: &mut MeshGraph,
        result: &mut MergeVerticesOneRing,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        self.log_rerun("add_to_mesh_graph", mesh_graph);

        // Transaction: validate before any mutation. Edges this face creates are
        // tracked so a failed insertion removes them again; `add_face` is the
        // commit point after which nothing can fail.
        self.validate_planned_face(mesh_graph)?;

        let mut created: Vec<HalfedgeId> = Vec::new();

        let add_or_get_edge1 = match mesh_graph.add_or_get_boundary_edge(self.v1, self.new_he_v1) {
            Some(edge) => edge,
            None => {
                // Unreachable after `validate_planned_face`.
                debug_assert!(false, "edge 1 acquisition failed after validation");
                return None;
            }
        };
        if add_or_get_edge1.new_start_to_end {
            created.push(add_or_get_edge1.start_to_end_he_id);
        }
        let add_or_get_edge2 =
            match mesh_graph.add_or_get_boundary_edge(self.new_he_v1, self.new_he_v2) {
                Some(edge) => edge,
                None => {
                    // Unreachable after `validate_planned_face`.
                    debug_assert!(false, "edge 2 acquisition failed after validation");
                    mesh_graph.rollback_created_edges(&mut created);
                    return None;
                }
            };
        if add_or_get_edge2.new_start_to_end {
            created.push(add_or_get_edge2.start_to_end_he_id);
        }

        // The re-filed halfedges may still belong to a live face (an overlapping
        // earlier merge's bridge or fan). Re-filing a live chain member would leave
        // the old face's chain referencing a halfedge that now claims a different
        // face (face-stealing corruption); dismantle such faces up front. They lie
        // inside the merged region and are rebuilt by the new faces.
        for he_id in [
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        ] {
            mesh_graph.dismantle_faced_member(he_id, result);
        }

        #[cfg(test)]
        mesh_graph.debug_assert_face_inputs_ready(
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        );

        let mut add_face = match mesh_graph.add_face_from_halfedges(
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        ) {
            Some(face) => face,
            None => {
                // Unreachable after the validation and readiness checks above;
                // fail loudly in tests instead of silently losing the face.
                debug_assert!(
                    false,
                    "planned face {self:?} failed after a clean transaction"
                );
                mesh_graph.rollback_created_edges(&mut created);
                return None;
            }
        };

        if add_or_get_edge1.new_start_to_end {
            add_face
                .halfedge_ids
                .push(add_or_get_edge1.start_to_end_he_id);
        }
        if add_or_get_edge1.new_twin {
            add_face.halfedge_ids.push(add_or_get_edge1.twin_he_id);
        }
        if add_or_get_edge2.new_start_to_end {
            add_face
                .halfedge_ids
                .push(add_or_get_edge2.start_to_end_he_id);
        }
        if add_or_get_edge2.new_twin {
            add_face.halfedge_ids.push(add_or_get_edge2.twin_he_id);
        }

        #[cfg(test)]
        mesh_graph.debug_assert_created_edges_claimed(&created);

        Some((
            if matches!(
                self.order,
                PlannedFaceOrder::Start | PlannedFaceOrder::Middle
            ) && mesh_graph.halfedges[add_or_get_edge2.twin_he_id].is_boundary()
            {
                Some(add_or_get_edge2.twin_he_id)
            } else {
                None
            },
            add_face,
        ))
    }

    #[instrument(skip(mesh_graph, result))]
    fn add_to_mesh_graph_and_he(
        &self,
        mesh_graph: &mut MeshGraph,
        existing_he_id: HalfedgeId,
        result: &mut MergeVerticesOneRing,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        self.log_rerun("add_to_mesh_graph_and_he", mesh_graph);

        // Transaction: validate before any mutation. `add_face` is the commit
        // point; everything before it is rolled back on failure (created edges)
        // or cannot be undone (dismantled faces) and must therefore be
        // infallible once entered.
        self.validate_planned_face(mesh_graph)?;

        if !matches!(self.order, PlannedFaceOrder::Middle | PlannedFaceOrder::End) {
            error!("Invalid face order {:?}", self.order);
            debug_assert!(false, "planned face with invalid order reached insertion");
            return None;
        }

        // The shared edge with the previous planned face. Capture its vertex
        // pair before the dismantle: `remove_face` can free both halves (member
        // and twin) when the other side was boundary, so an id resolved here can
        // be dead afterwards. The edge is re-resolved - or re-created - by its
        // vertex pair after the dismantle.
        let existing_edge = mesh_graph.halfedge_endpoints(existing_he_id)?;

        let mut created: Vec<HalfedgeId> = Vec::new();

        // The edge may already exist (created by the previous planned face
        // of this merge or an overlapping earlier merge). Take that one
        // instead of adding a second edge between the same two vertices:
        // `add_edge` here used to leave a *parallel* halfedge pair, whose
        // two free halves stay face-less forever - the mesh reports them
        // as boundary and the weld looks like it left a hole, while the
        // two triangles that should have shared the edge each sit on their
        // own copy of it. (`End` below has always done it this way.)
        let add_or_get_edge =
            match mesh_graph.add_or_get_boundary_edge(self.new_he_v1, self.new_he_v2) {
                Some(edge) => edge,
                None => {
                    debug_assert!(
                        false,
                        "edge acquisition failed after `validate_planned_face`"
                    );
                    mesh_graph.rollback_created_edges(&mut created);
                    return None;
                }
            };
        if add_or_get_edge.new_start_to_end {
            created.push(add_or_get_edge.start_to_end_he_id);
        }

        // A re-filed halfedge may still belong to a live face's chain.
        // Dismantle the claiming face before re-filing, and the existing
        // edge (the other side of the shared border) likewise, so no live
        // chain keeps referencing a re-filed halfedge.
        mesh_graph.dismantle_faced_member(existing_he_id, result);
        mesh_graph.dismantle_faced_member(add_or_get_edge.start_to_end_he_id, result);

        // Re-resolve the shared edge after the dismantle.
        let existing_he_id = match mesh_graph
            .boundary_half_from_to(existing_edge.0, existing_edge.1)
            .or_else(|| mesh_graph.boundary_half_from_to(existing_edge.1, existing_edge.0))
        {
            Some(he_id) => he_id,
            None => {
                // Both halves went with the dismantled face: re-create the edge.
                match mesh_graph.add_or_get_boundary_edge(existing_edge.0, existing_edge.1) {
                    Some(edge) => {
                        if edge.new_start_to_end {
                            created.push(edge.start_to_end_he_id);
                        }
                        edge.start_to_end_he_id
                    }
                    None => {
                        // Unreachable after `validate_planned_face`.
                        debug_assert!(
                            false,
                            "re-creating the shared edge failed after a clean transaction"
                        );
                        mesh_graph.rollback_created_edges(&mut created);
                        return None;
                    }
                }
            }
        };

        #[cfg(test)]
        mesh_graph
            .debug_assert_face_inputs_ready(existing_he_id, add_or_get_edge.start_to_end_he_id);

        let mut add_face = match mesh_graph
            .add_face_from_halfedges(existing_he_id, add_or_get_edge.start_to_end_he_id)
        {
            Some(face) => face,
            None => {
                // Unreachable after the dismantle + re-resolution + readiness
                // checks above; the dismantled faces cannot be restored, so
                // fail loudly in tests rather than silently losing the face.
                debug_assert!(
                    false,
                    "planned face {self:?} failed after irreversible dismantles"
                );
                mesh_graph.rollback_created_edges(&mut created);
                return None;
            }
        };

        if add_or_get_edge.new_start_to_end {
            add_face
                .halfedge_ids
                .push(add_or_get_edge.start_to_end_he_id);
        }
        if add_or_get_edge.new_twin {
            add_face.halfedge_ids.push(add_or_get_edge.twin_he_id);
        }

        #[cfg(test)]
        mesh_graph.debug_assert_created_edges_claimed(&created);

        if matches!(self.order, PlannedFaceOrder::Middle) {
            Some((Some(add_or_get_edge.twin_he_id), add_face))
        } else {
            Some((None, add_face))
        }
    }

    #[cfg(feature = "rerun")]
    fn log_rerun(&self, label: &str, mesh_graph: &MeshGraph) {
        use crate::{RR, utils::vec3_array};

        let (positions, labels): (Vec<_>, Vec<_>) =
            [(self.v1, "1"), (self.new_he_v1, "2"), (self.new_he_v2, "3")]
                .into_iter()
                .filter_map(|(v_id, labl)| mesh_graph.positions.get(v_id).map(|a| (a, labl)))
                .unzip();

        RR.log(
            format!("meshgraph/planned_face/{label}/positions"),
            &rerun::Points3D::new(positions.iter().map(vec3_array)).with_labels(labels),
        )
        .unwrap();

        RR.log(
            format!("meshgraph/planned_face/{label}/edges"),
            &rerun::Arrows3D::from_vectors(
                positions
                    .iter()
                    .circular_array_windows()
                    .map(|[a, b]| vec3_array(*b - *a)),
            )
            .with_origins(positions.iter().map(vec3_array)),
        )
        .unwrap();
    }
}
