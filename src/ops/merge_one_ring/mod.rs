#[cfg(test)]
mod tests;

use std::{f32, ops::RangeInclusive};

use hashbrown::HashSet;
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    AddEdge, FaceId, HalfedgeId, MeshGraph, Vertex, VertexId, error_none,
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
        let mut result = MergeVerticesOneRing::default();

        let vertex1 = *unwrap_or_return!(self.vertices.get(vertex_id1), "Vertex not found", result);
        let vertex2 = *unwrap_or_return!(self.vertices.get(vertex_id2), "Vertex not found", result);

        let one_ring_he_ids1 = vertex1.one_ring(self).collect_vec();
        let mut one_ring_he_ids2 = vertex2
            .one_ring(self)
            // halfedges are reversed, so twin halfedges are actually the next halfedges.
            // also existence is checked in `one_ring()`.
            .filter_map(|he_id| {
                self.halfedges[he_id]
                    .twin
                    .or_else(error_none!("Twin not found"))
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
            .map(|he_id| self.halfedges[*he_id].end_vertex)
            .collect_vec();
        one_ring_v_ids1.rotate_right(1);
        let mut one_ring_v_ids2 = one_ring_he_ids2
            .iter()
            .map(|he_id| self.halfedges[*he_id].end_vertex)
            .collect_vec();
        one_ring_v_ids2.rotate_right(1);

        if one_ring_v_ids1.len() < 3 || one_ring_v_ids2.len() < 3 {
            error!("One rings are too small");
            return result;
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

        if self.check_and_flip_single_shared_he(&shared_he_ids, flip_threshold_sqr, &mut result) {
            return result;
        }

        self.remove_neighbor_faces(&vertex1, &vertex2, &mut result);

        #[cfg(feature = "rerun")]
        self.log_rerun();

        let (already_connected_face_ids, connected_v_ids, connected_he_ids) = unwrap_or_return!(
            self.find_already_connected_pairings(
                &one_ring_v_ids1,
                &one_ring_v_ids2,
                &shared_v_ids,
            ),
            "Error in find_already_connected_pairings",
            result
        );

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

        if range_pairs_to_connect.is_empty() {
            error!("No range pairs to connect");
            return result;
        }

        tracing::info!("Range pairs to connect: {range_pairs_to_connect:#?}");

        let planned_faces =
            self.plan_new_faces(&range_pairs_to_connect, &one_ring_v_ids1, &one_ring_v_ids2);

        for face_id in already_connected_face_ids {
            let (v_ids, he_ids) = self.remove_face(face_id);
            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(he_ids);
            result.removed_vertices.extend(v_ids);
        }

        self.add_planned_faces(
            planned_faces,
            &one_ring_v_set1,
            &one_ring_v_set2,
            marked_halfedges,
            &mut result,
        );

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

        self.smooth_vertices(
            one_ring_v_ids1
                .iter()
                .chain(&one_ring_v_ids2)
                .chain(&result.added_vertices)
                .copied(),
        );

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

            if let Some(he_id) = self.halfedge_from_to(v_id1, v_id2) {
                let pairing = self.find_triangle_fan(
                    he_id,
                    idx1,
                    idx2,
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &mut connected_he_ids,
                    &mut already_connected_face_ids,
                )?;

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
    ) -> Option<Pairing> {
        let he = self
            .halfedges
            .get(he_id)
            .or_else(error_none!("Halfedge not found"))?;

        let mut current_pairing = Pairing {
            single_range_idx: 0,
            single_idx_in_range: idx1,
            other_range: idx2..=idx2,
        };

        // might not exist. we removed some faces (neighbors).
        let Some(opposite_v_id) = he.opposite_vertex(self) else {
            return Some(current_pairing);
        };

        if !self.create_triangle_pairing(
            one_ring_v_ids1,
            one_ring_v_ids2,
            idx1,
            idx2,
            opposite_v_id,
            &mut current_pairing,
        ) {
            let twin_id = he.twin.or_else(error_none!("Twin not found"))?;
            let twin = self
                .halfedges
                .get(twin_id)
                .or_else(error_none!("Twin not found"))?;

            // might not exist. we removed some faces (neighbors).
            let Some(opposite_v_id) = twin.opposite_vertex(self) else {
                return Some(current_pairing);
            };

            if !self.create_triangle_pairing(
                one_ring_v_ids1,
                one_ring_v_ids2,
                idx1,
                idx2,
                opposite_v_id,
                &mut current_pairing,
            ) {
                // Return single halfedge current pairing
                return Some(current_pairing);
            }
        }

        self.extend_triangle_pairing_fan(
            one_ring_v_ids1,
            one_ring_v_ids2,
            &mut current_pairing,
            connected_he_ids,
            connected_face_ids,
        );

        Some(current_pairing)
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
            } else {
                break;
            }
        }

        // Walk forwards from the end of other_range
        let mut range_end = *current_pairing.other_range.end();
        loop {
            let next_idx = (range_end + 1) % other_len;
            let next_v_id = other_ids[next_idx];

            let boundary_v_id = other_ids[range_end % other_len];
            if let Some(face_id) = self.face_with_vertices(single_v_id, boundary_v_id, next_v_id) {
                range_end += 1;
                current_pairing.other_range = *current_pairing.other_range.start()..=range_end;

                self.add_face_to_connected_he_ids(face_id, connected_he_ids);
                connected_face_ids.push(face_id);
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
                idx1.min(idx)..=idx1.max(idx);

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
                planned_face.add_to_mesh_graph_and_he(self, prev_he)
            } else {
                planned_face.add_to_mesh_graph(self)
            };

            let Some(inserted) = inserted else {
                // depending on the topology inserting a face can fail sometimes. this is fine.
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

    fn remove_neighbor_faces(
        &mut self,
        vertex1: &Vertex,
        vertex2: &Vertex,
        result: &mut MergeVerticesOneRing,
    ) {
        let face_ids1 = vertex1.faces(self).collect_vec();
        let face_ids2 = vertex2.faces(self).collect_vec();

        for face_id in face_ids1.into_iter().chain(face_ids2) {
            let (del_v, del_he) = self.remove_face(face_id);

            result.removed_faces.push(face_id);
            result.removed_vertices.extend(del_v);
            result.removed_halfedges.extend(del_he);
        }
    }

    #[instrument(skip_all)]
    fn flip_and_collapse_single_shared_edge_if_below_threshold(
        &mut self,
        single_shared_he_id: HalfedgeId,
        flip_threshold_sqr: f32,
        result: &mut MergeVerticesOneRing,
    ) -> bool {
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
            tracing::info!("Flipping edge {single_shared_he_id:?}");

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

            tracing::info!("Pairings: {:#?}", pairings);

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

        tracing::info!(
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

        while end_idx1 != orig_start_idx1 {
            v_id1 = one_ring_v_ids1[end_idx1];
            v_id2 = one_ring_v_ids2[end_idx2];

            if !self.vertices.contains_key(v_id1) || !self.vertices.contains_key(v_id2) {
                break;
            }

            let mut end_cap = ConnectPairCap::Open;

            if shared_v_ids.contains(&v_id1) {
                while v_id2 != v_id1 {
                    end_idx2 = (end_idx2 + 1) % len2;
                    v_id2 = one_ring_v_ids2[end_idx2];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx2", v_id2);
                }

                end_cap = ConnectPairCap::Closed;
            } else if shared_v_ids.contains(&v_id2) {
                while v_id1 != v_id2 {
                    end_idx1 = (end_idx1 + 1) % len1;
                    v_id1 = one_ring_v_ids1[end_idx1];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx1", v_id1);
                }

                end_cap = ConnectPairCap::Closed;
            } else if connected_v_ids.contains(&v_id1) {
                while !connected_v_ids.contains(&v_id2) {
                    end_idx2 = (end_idx2 + 1) % len2;
                    v_id2 = one_ring_v_ids2[end_idx2];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx2", v_id2);
                }

                end_cap = ConnectPairCap::AlreadyConnected;
            } else if connected_v_ids.contains(&v_id2) {
                while !connected_v_ids.contains(&v_id1) {
                    end_idx1 = (end_idx1 + 1) % len1;
                    v_id1 = one_ring_v_ids1[end_idx1];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx1", v_id1);
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

        let diff1 = (start_idx1 as i32 - end_idx1 as i32).unsigned_abs() as usize;
        let diff1 = diff1.min(len1 - diff1);

        let diff2 = (start_idx2 as i32 - end_idx2 as i32).unsigned_abs() as usize;
        let diff2 = diff2.min(len2 - diff2);

        let cap = if shared_v_ids.is_empty() {
            ConnectPairCap::Open
        } else {
            ConnectPairCap::Closed
        };

        if range_pairs_to_connect.is_empty() {
            let (end1, end2) = if matches!(cap, ConnectPairCap::Closed) {
                (orig_start_idx1, orig_start_idx2)
            } else {
                (
                    (orig_start_idx1 + len1 - 1).rem_euclid(len1),
                    (orig_start_idx2 + len2 - 1).rem_euclid(len2),
                )
            };

            range_pairs_to_connect.push(ConnectPair::new(
                orig_start_idx1..=end1,
                len1,
                orig_start_idx2..=end2,
                len2,
                cap,
            ));
        } else if diff1 > 1 || diff2 > 1 || range_pairs_to_connect.is_empty() {
            range_pairs_to_connect.push(ConnectPair::new(
                start_idx1..=end_idx1,
                len1,
                start_idx2..=end_idx2,
                len2,
                cap,
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
            if self.halfedges.contains_key(he_id1)
                && !shared_he_ids.contains(&he_id1)
                && !connected_he_ids.contains(&he_id1)
            {
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
        let other_range = if (other_idx1 as i32 - other_idx2 as i32).abs() == 1 {
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
                &rerun::Points3D::new([single_pos.clone()]),
            )
            .unwrap();
        crate::RR
            .log(
                format!("pairing/{}/other", label.as_ref()),
                &rerun::LineStrips3D::new(
                    other_pos
                        .into_iter()
                        .map(|other_pos| [single_pos.clone(), other_pos]),
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

    #[instrument(skip(mesh_graph))]
    fn add_to_mesh_graph(
        &self,
        mesh_graph: &mut MeshGraph,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        self.log_rerun("add_to_mesh_graph", mesh_graph);

        let add_or_get_edge1 = mesh_graph.add_or_get_boundary_edge(self.v1, self.new_he_v1)?;
        let add_or_get_edge2 =
            mesh_graph.add_or_get_boundary_edge(self.new_he_v1, self.new_he_v2)?;

        let mut add_face = mesh_graph.add_face_from_halfedges(
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        )?;

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

        Some((
            if matches!(
                self.order,
                PlannedFaceOrder::Start | PlannedFaceOrder::Middle
            ) {
                Some(add_or_get_edge2.twin_he_id)
            } else {
                None
            },
            add_face,
        ))
    }

    #[instrument(skip(mesh_graph))]
    fn add_to_mesh_graph_and_he(
        &self,
        mesh_graph: &mut MeshGraph,
        existing_he_id: HalfedgeId,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        self.log_rerun("add_to_mesh_graph_and_he", mesh_graph);

        match self.order {
            PlannedFaceOrder::Middle => {
                let AddEdge {
                    start_to_end_he_id,
                    twin_he_id,
                } = mesh_graph.add_edge(self.new_he_v1, self.new_he_v2)?;

                let mut add_face =
                    mesh_graph.add_face_from_halfedges(existing_he_id, start_to_end_he_id)?;
                add_face.halfedge_ids.push(start_to_end_he_id);
                add_face.halfedge_ids.push(twin_he_id);

                Some((Some(twin_he_id), add_face))
            }
            PlannedFaceOrder::End => {
                let add_or_get_edge =
                    mesh_graph.add_or_get_boundary_edge(self.new_he_v1, self.new_he_v2)?;

                let mut add_face = mesh_graph
                    .add_face_from_halfedges(existing_he_id, add_or_get_edge.start_to_end_he_id)?;
                if add_or_get_edge.new_start_to_end {
                    add_face
                        .halfedge_ids
                        .push(add_or_get_edge.start_to_end_he_id);
                }
                if add_or_get_edge.new_twin {
                    add_face.halfedge_ids.push(add_or_get_edge.twin_he_id);
                }

                Some((None, add_face))
            }
            _ => {
                error!("Invalid face order {:?}", self.order);
                None
            }
        }
    }

    #[cfg(feature = "rerun")]
    fn log_rerun(&self, label: &str, mesh_graph: &MeshGraph) {
        use crate::{RR, utils::vec3_array};

        let (positions, labels): (Vec<_>, Vec<_>) =
            [(self.v1, "1"), (self.new_he_v1, "2"), (self.new_he_v2, "3")]
                .into_iter()
                .filter_map(|(v_id, labl)| {
                    mesh_graph.positions.get(v_id).and_then(|a| Some((a, labl)))
                })
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
                    .circular_tuple_windows()
                    .map(|(a, b)| vec3_array(*b - *a)),
            )
            .with_origins(positions.iter().map(vec3_array)),
        )
        .unwrap();
    }
}
