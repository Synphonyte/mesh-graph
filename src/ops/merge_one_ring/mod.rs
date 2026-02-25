#[cfg(test)]
mod tests;

use std::{f32, ops::RangeInclusive};

use hashbrown::HashSet;
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    AddEdge, AddOrGetEdge, FaceId, HalfedgeId, MeshGraph, Vertex, VertexId, error_none,
    ops::{add::AddFace, collapse::CollapseEdge},
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

        // halfedges' existence already checked in `one_ring()`.
        let one_ring_v_ids1 = vertex1
            .one_ring(self)
            .map(|he_id| self.halfedges[he_id].end_vertex)
            .collect_vec();
        let mut one_ring_v_ids2 = vertex2
            .one_ring(self)
            .map(|he_id| self.halfedges[he_id].end_vertex)
            .collect_vec();
        one_ring_v_ids2.reverse();

        if one_ring_v_ids1.len() < 3 || one_ring_v_ids2.len() < 3 {
            error!("One rings are too small");
            return result;
        }

        let one_ring_set1 = HashSet::<VertexId>::from_iter(one_ring_v_ids1.iter().copied());
        let one_ring_set2 = HashSet::<VertexId>::from_iter(one_ring_v_ids2.iter().copied());
        let shared_v_ids =
            HashSet::<VertexId>::from_iter(one_ring_set1.intersection(&one_ring_set2).copied());

        if self.check_and_flip_single_shared_he(
            &one_ring_v_ids1,
            &one_ring_v_ids2,
            &shared_v_ids,
            flip_threshold_sqr,
            &mut result,
        ) {
            return result;
        }

        self.remove_neighbour_faces(&vertex1, &vertex2, &mut result);

        #[cfg(feature = "rerun")]
        self.log_rerun();

        let range_pairs_to_connect =
            self.compute_range_pairs_to_connect(&one_ring_v_ids1, &one_ring_v_ids2, &shared_v_ids);

        if range_pairs_to_connect.is_empty() {
            error!("No range pairs to connect");
            return result;
        }

        tracing::info!("Range pairs to connect: {range_pairs_to_connect:#?}");

        let planned_faces =
            self.plan_new_faces(&range_pairs_to_connect, &one_ring_v_ids1, &one_ring_v_ids2);

        self.add_planned_faces(
            planned_faces,
            &one_ring_set1,
            &one_ring_set2,
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

        tracing::info!("smoothing vertices");
        self.smooth_vertices(
            one_ring_v_ids1
                .iter()
                .chain(&one_ring_v_ids2)
                .chain(&result.added_vertices)
                .copied(),
        );

        result
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
                planned_face.add_to_meshgraph_and_he(self, prev_he)
            } else {
                planned_face.add_to_meshgraph(self)
            };

            let inserted = unwrap_or_return!(inserted, "Failed to add face");

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

    fn remove_neighbour_faces(
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

            if !range_pair_to_connect.closed {
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

            planned_faces[start_pairing_index].order = PlannedFaceOrder::Start;

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
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        shared_v_ids: &HashSet<VertexId>,
        flip_threshold_sqr: f32,
        result: &mut MergeVerticesOneRing,
    ) -> bool {
        if shared_v_ids.len() == 2 {
            let (first_v_id, second_v_id) = shared_v_ids.iter().copied().collect_tuple().unwrap();

            let first_pos1 = one_ring_v_ids1
                .iter()
                .position(|v_id| *v_id == first_v_id)
                .unwrap() as i64;
            let second_pos1 = one_ring_v_ids1
                .iter()
                .position(|v_id| *v_id == second_v_id)
                .unwrap() as i64;

            let first_pos2 = one_ring_v_ids2
                .iter()
                .position(|v_id| *v_id == first_v_id)
                .unwrap() as i64;
            let second_pos2 = one_ring_v_ids2
                .iter()
                .position(|v_id| *v_id == second_v_id)
                .unwrap() as i64;

            if (first_pos1 - second_pos1).abs() != 1 || (first_pos2 - second_pos2).abs() != 1 {
                return false;
            }

            #[cfg(feature = "rerun")]
            self.log_verts_rerun("common_one_ring", &[first_v_id, second_v_id]);

            if let Some(he_id) = self.halfedge_from_to(first_v_id, second_v_id) {
                #[cfg(feature = "rerun")]
                self.log_he_rerun("common_one_ring_he", he_id);

                return self.flip_and_collapse_single_shared_edge_if_below_threshold(
                    he_id,
                    flip_threshold_sqr,
                    result,
                );
            }
        }

        false
    }

    #[instrument(skip_all)]
    fn compute_range_pairs_to_connect(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        shared_v_ids: &HashSet<VertexId>,
    ) -> Vec<ConnectPair> {
        let mut range_pairs_to_connect = vec![];

        let mut connected_v_ids = HashSet::new();

        let (orig_start_idx1, orig_start_idx2) = unwrap_or_return!(
            self.find_start_indices(
                one_ring_v_ids1,
                one_ring_v_ids2,
                shared_v_ids,
                &connected_v_ids
            ),
            "Couldn't find start indices",
            range_pairs_to_connect
        );

        tracing::info!(
            "start idx1: {}, start idx2: {}",
            orig_start_idx1,
            orig_start_idx2
        );

        if orig_start_idx1 != 0 || orig_start_idx2 != 5 {
            println!("nope");
        }

        let len1 = one_ring_v_ids1.len();
        let len2 = one_ring_v_ids2.len();

        let mut start_idx1 = orig_start_idx1;
        let mut start_idx2 = orig_start_idx2;

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

            let mut shared = false;

            if shared_v_ids.contains(&v_id1) {
                while v_id2 != v_id1 {
                    end_idx2 = (end_idx2 + 1) % len2;
                    v_id2 = one_ring_v_ids2[end_idx2];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx2", v_id2);
                }

                shared = true;
            } else if shared_v_ids.contains(&v_id2) {
                while v_id1 != v_id2 {
                    end_idx1 = (end_idx1 + 1) % len1;
                    v_id1 = one_ring_v_ids1[end_idx1];

                    #[cfg(feature = "rerun")]
                    self.log_vert_rerun("pairs_end_idx1", v_id1);
                }

                shared = true;
            }

            if shared {
                range_pairs_to_connect.push(ConnectPair::new(
                    start_idx1..=end_idx1,
                    len1,
                    start_idx2..=end_idx2,
                    len2,
                    true,
                ));

                let mut idx = start_idx1;
                while idx != end_idx1 {
                    connected_v_ids.insert(one_ring_v_ids1[idx]);
                    idx = (idx + 1) % len1;
                }
                idx = start_idx2;
                while idx != end_idx2 {
                    connected_v_ids.insert(one_ring_v_ids2[idx]);
                    idx = (idx + 1) % len2;
                }

                if let Some((idx1, idx2)) = self.find_start_indices(
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    shared_v_ids,
                    &connected_v_ids,
                ) {
                    start_idx1 = idx1;
                    start_idx2 = idx2;
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

        let closed = !shared_v_ids.is_empty();

        if range_pairs_to_connect.is_empty() {
            let (end1, end2) = if closed {
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
                closed,
            ));
        } else if diff1 > 1 || diff2 > 1 || range_pairs_to_connect.is_empty() {
            range_pairs_to_connect.push(ConnectPair::new(
                start_idx1..=end_idx1,
                len1,
                start_idx2..=end_idx2,
                len2,
                closed,
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
    fn find_shared_start_indices_from_ring(
        &self,
        one_ring_v_ids: &[VertexId],
        other_one_ring_v_ids: &[VertexId],
        shared_v_ids: &HashSet<VertexId>,
        connected_v_ids: &HashSet<VertexId>,
    ) -> Option<(usize, usize)> {
        let len = one_ring_v_ids.len();

        for (idx, &v_id1) in one_ring_v_ids.iter().enumerate() {
            if self.vertices.contains_key(v_id1)
                && !shared_v_ids.contains(&v_id1)
                && !connected_v_ids.contains(&v_id1)
            {
                let mut start_idx = idx;
                let mut start_v_id = &v_id1;

                while self.vertices.contains_key(*start_v_id)
                    && !shared_v_ids.contains(start_v_id)
                    && !connected_v_ids.contains(start_v_id)
                {
                    start_idx = (start_idx + len - 1) % len;
                    start_v_id = &one_ring_v_ids[start_idx];

                    if start_idx == idx {
                        return None;
                    }
                }

                return Some((
                    start_idx,
                    other_one_ring_v_ids
                        .iter()
                        .position(|v_id| start_v_id == v_id)
                        .unwrap(),
                ));
            }
        }

        None
    }

    #[instrument(skip(self))]
    fn find_start_indices(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        shared_v_ids: &HashSet<VertexId>,
        connected_v_ids: &HashSet<VertexId>,
    ) -> Option<(usize, usize)> {
        if !shared_v_ids.is_empty() {
            if let Some(indices) = self.find_shared_start_indices_from_ring(
                one_ring_v_ids1,
                one_ring_v_ids2,
                shared_v_ids,
                connected_v_ids,
            ) {
                Some(indices)
            } else if let Some((idx2, idx1)) = self.find_shared_start_indices_from_ring(
                one_ring_v_ids2,
                one_ring_v_ids1,
                shared_v_ids,
                connected_v_ids,
            ) {
                Some((idx1, idx2))
            } else {
                None
            }
        } else {
            for (idx1, &v_id1) in one_ring_v_ids1.iter().enumerate() {
                for (idx2, &v_id2) in one_ring_v_ids2.iter().enumerate() {
                    if self.halfedge_from_to(v_id1, v_id2).is_some() {
                        tracing::info!(
                            "Found common edge between vertices {idx1} (id {v_id1:?}) and {idx2} (id {v_id2:?})"
                        );
                        return Some((idx1, idx2));
                    }
                }
            }

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

            Some((0, start_idx2))
        }
    }
}

/// Two ranges of elements that need to be connected by faces.
#[derive(Debug)]
struct ConnectPair {
    /// Wether the first and last indices in the two ranges reference the same vertex, thus forming a closed polygon.
    closed: bool,

    /// The pair of ranges of vertex indices that need to be connected.
    ranges: [RangeInclusive<usize>; 2],
}

impl ConnectPair {
    fn new(
        mut range1: RangeInclusive<usize>,
        len1: usize,
        mut range2: RangeInclusive<usize>,
        len2: usize,
        closed: bool,
    ) -> Self {
        if range1.end() <= range1.start() {
            range1 = (*range1.start())..=(*range1.end() + len1);
        }
        if range2.end() <= range2.start() {
            range2 = (*range2.start())..=(*range2.end() + len2);
        }

        ConnectPair {
            closed,
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
#[derive(Debug)]
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
        let single_ids = &v_ids[self.single_range_idx];
        let other_ids = &v_ids[1 - self.single_range_idx];

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let mut faces = Vec::<PlannedFace>::new();

        let mut others = self.other_range.clone();

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
}

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
    fn add_to_meshgraph(
        &self,
        mesh_graph: &mut MeshGraph,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        mesh_graph.log_verts_w_labels_rerun(
            "add_to_meshgraph",
            &[self.v1, self.new_he_v1, self.new_he_v2],
            &["1", "2", "3"],
        );

        let add_or_get_edge = mesh_graph.add_or_get_edge(self.v1, self.new_he_v1);

        let (start_to_end_he_id, twin_he_id) = match self.order {
            PlannedFaceOrder::Start => {
                let AddEdge {
                    start_to_end_he_id,
                    twin_he_id,
                } = mesh_graph.add_edge(self.new_he_v1, self.new_he_v2);

                (start_to_end_he_id, twin_he_id)
            }
            PlannedFaceOrder::Single => (
                mesh_graph.halfedge_from_to(self.new_he_v1, self.new_he_v2)?,
                mesh_graph.halfedge_from_to(self.new_he_v2, self.new_he_v1)?,
            ),
            _ => {
                error!("Invalid face order {:?}", self.order);
                return None;
            }
        };

        let he = mesh_graph
            .halfedges
            .get(add_or_get_edge.start_to_end_he_id)
            .or_else(error_none!("Halfedge not found"))?;

        let add_or_get_edge = if he.is_boundary() {
            add_or_get_edge
        } else {
            let AddEdge {
                start_to_end_he_id,
                twin_he_id,
            } = mesh_graph.add_edge(self.v1, self.new_he_v1);

            AddOrGetEdge {
                start_to_end_he_id,
                twin_he_id,
                new_start_to_end: true,
                new_twin: true,
            }
        };

        let mut add_face = mesh_graph
            .add_face_from_halfedges(add_or_get_edge.start_to_end_he_id, start_to_end_he_id)?;

        add_face.halfedge_ids.push(start_to_end_he_id);
        add_face.halfedge_ids.push(twin_he_id);

        if add_or_get_edge.new_start_to_end {
            add_face
                .halfedge_ids
                .push(add_or_get_edge.start_to_end_he_id);
        }
        if add_or_get_edge.new_twin {
            add_face.halfedge_ids.push(add_or_get_edge.twin_he_id);
        }

        Some((
            if matches!(self.order, PlannedFaceOrder::Start) {
                Some(twin_he_id)
            } else {
                None
            },
            add_face,
        ))
    }

    #[instrument(skip(mesh_graph))]
    fn add_to_meshgraph_and_he(
        &self,
        mesh_graph: &mut MeshGraph,
        existing_he_id: HalfedgeId,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_he_rerun("add_meshgraph_and_he", existing_he_id);
            mesh_graph.log_verts_w_labels_rerun(
                "add_to_meshgraph_and_he",
                &[self.v1, self.new_he_v1, self.new_he_v2],
                &["1", "2", "3"],
            );
        }

        match self.order {
            PlannedFaceOrder::Middle => {
                let AddEdge {
                    start_to_end_he_id,
                    twin_he_id,
                } = mesh_graph.add_edge(self.new_he_v1, self.new_he_v2);

                let mut add_face =
                    mesh_graph.add_face_from_halfedges(existing_he_id, start_to_end_he_id)?;
                add_face.halfedge_ids.push(start_to_end_he_id);
                add_face.halfedge_ids.push(twin_he_id);

                Some((Some(twin_he_id), add_face))
            }
            PlannedFaceOrder::End => {
                let end_he = mesh_graph
                    .halfedge_from_to(self.new_he_v1, self.new_he_v2)
                    .or_else(error_none!("Final halfedge not found"))?;

                let add_face = mesh_graph.add_face_from_halfedges(existing_he_id, end_he)?;

                Some((None, add_face))
            }
            _ => {
                error!("Invalid face order {:?}", self.order);
                None
            }
        }
    }
}
