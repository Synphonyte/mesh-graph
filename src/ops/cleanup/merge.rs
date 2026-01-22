use std::{f32, ops::RangeInclusive};

use glam::Vec3;
use hashbrown::HashSet;
use itertools::Itertools;
use slotmap::SecondaryMap;
use tracing::{error, instrument};

use crate::{
    FaceId, HalfedgeId, MeshGraph, Vertex, VertexId, error_none, ops::create::CreateFace,
    utils::unwrap_or_return,
};

#[derive(Default)]
pub struct MergeVerticesOneRing {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

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
    #[instrument(skip(self))]
    pub fn merge_vertices_one_rings(
        &mut self,
        vertex_id1: VertexId,
        vertex_id2: VertexId,
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

        let range_pairs_to_connect =
            self.compute_range_pairs_to_connect(&one_ring_v_ids1, &one_ring_v_ids2);

        tracing::info!("Range pairs to connect: {:#?}", range_pairs_to_connect);

        let planned_faces =
            self.plan_new_faces(range_pairs_to_connect, &one_ring_v_ids1, &one_ring_v_ids2);

        let mut vertex_normals = SecondaryMap::new();

        self.delete_neighbour_faces_and_estimate_vertex_normals(
            &vertex1,
            &vertex2,
            &mut vertex_normals,
            &mut result,
        );

        #[cfg(feature = "rerun")]
        self.log_rerun();

        tracing::info!("Planned Faces: {:#?}", planned_faces);

        for mut planned_face in planned_faces {
            planned_face.make_ccw(&vertex_normals, self);

            let inserted = unwrap_or_return!(
                planned_face.insert_into_meshgraph(self),
                "Couldn't create face",
                result
            );

            result.added_faces.push(inserted.face_id);
            result.added_halfedges.extend(inserted.halfedge_ids);
        }

        result
    }

    fn delete_neighbour_faces_and_estimate_vertex_normals(
        &mut self,
        vertex1: &Vertex,
        vertex2: &Vertex,
        vertex_normals: &mut SecondaryMap<VertexId, Vec3>,
        result: &mut MergeVerticesOneRing,
    ) {
        let face_ids1 = vertex1.faces(self).collect_vec();
        let face_ids2 = vertex2.faces(self).collect_vec();

        for face_id in face_ids1.into_iter().chain(face_ids2) {
            let face = unwrap_or_return!(self.faces.get(face_id), "Face not found");
            for v_id in face.vertices(self) {
                if let Some(normals) = &self.vertex_normals {
                    vertex_normals.insert(
                        v_id,
                        *unwrap_or_return!(normals.get(v_id), "Normal not found"),
                    );
                } else {
                    let normal = unwrap_or_return!(face.normal(self), "Normal not found");
                    vertex_normals.insert(v_id, normal);
                }
            }

            let (del_v, del_he) = self.delete_face(face_id);

            result.removed_faces.push(face_id);
            result.removed_vertices.extend(del_v);
            result.removed_halfedges.extend(del_he);
        }
    }

    fn plan_new_faces(
        &self,
        range_pairs_to_connect: Vec<ConnectPair>,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
    ) -> Vec<PlannedFace> {
        let mut planned_faces = Vec::with_capacity(one_ring_v_ids1.len());

        let len1 = one_ring_v_ids1.len();
        let len2 = one_ring_v_ids2.len();

        for range_pair_to_connect in range_pairs_to_connect {
            let pairings = range_pair_to_connect.compute_pairings();

            if pairings.len() < 2 {
                // TODO : implement
                continue;
            }

            tracing::info!("Pairings: {:#?}", pairings);

            let mut prev_single_v_id = None;
            let mut prev_other_v_id = None;

            if range_pair_to_connect.closed {
                // First and last vertices are identical => connect them with the first separated pair of vertices with a triangle.
                //
                // TODO : how to make sure that they're CCW?
                let start1_idx = *range_pair_to_connect.ranges[0].start() % len1;
                planned_faces.push(PlannedFace::new(
                    one_ring_v_ids1[start1_idx],
                    one_ring_v_ids1[(start1_idx + 1) % len1],
                    one_ring_v_ids2[(*range_pair_to_connect.ranges[1].start() + 1) % len2],
                ));

                let end1_idx = *range_pair_to_connect.ranges[0].end() % len1;
                planned_faces.push(PlannedFace::new(
                    one_ring_v_ids1[(end1_idx - 1).rem_euclid(len1)],
                    one_ring_v_ids1[end1_idx],
                    one_ring_v_ids2[(*range_pair_to_connect.ranges[1].end() - 1).rem_euclid(len2)],
                ));
            } else {
                let last_pairing = pairings.last().unwrap();

                let (s, o) = last_pairing.last_pair([one_ring_v_ids1, one_ring_v_ids2]);

                prev_single_v_id = Some(s);
                prev_other_v_id = Some(o);
            }

            for pairing in pairings {
                if let Some(prev_single_idx) = prev_single_v_id
                    && let Some(prev_other_idx) = prev_other_v_id
                {
                    // add the quad between the previous and current pairings
                    let (single_v_id, other_v_id) =
                        pairing.first_pair([one_ring_v_ids1, one_ring_v_ids2]);

                    planned_faces.push(PlannedFace::new(
                        prev_single_idx,
                        single_v_id,
                        prev_other_idx,
                    ));
                    planned_faces.push(PlannedFace::new(prev_other_idx, single_v_id, other_v_id));
                }

                // add the faces of the current pairing
                planned_faces.extend(pairing.fill_faces([one_ring_v_ids1, one_ring_v_ids2]));

                let (single_v_id, other_v_id) =
                    pairing.last_pair([one_ring_v_ids1, one_ring_v_ids2]);

                prev_single_v_id = Some(single_v_id);
                prev_other_v_id = Some(other_v_id);
            }
        }

        planned_faces
    }

    fn compute_range_pairs_to_connect(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
    ) -> Vec<ConnectPair> {
        let mut range_pairs_to_connect = vec![];

        let one_ring_set1 = HashSet::<VertexId>::from_iter(one_ring_v_ids1.iter().copied());
        let one_ring_set2 = HashSet::<VertexId>::from_iter(one_ring_v_ids2.iter().copied());
        let common_v_ids =
            HashSet::<VertexId>::from_iter(one_ring_set1.intersection(&one_ring_set2).copied());

        let (orig_start_idx1, orig_start_idx2) = unwrap_or_return!(
            self.find_start_indices(one_ring_v_ids1, one_ring_v_ids2, &common_v_ids),
            "Couldn't find start indices",
            range_pairs_to_connect
        );

        #[cfg(feature = "rerun")]
        {
            self.log_vert_rerun("start_v_1", one_ring_v_ids1[orig_start_idx1]);
            self.log_vert_rerun("start_v_2", one_ring_v_ids2[orig_start_idx2]);
        }

        let len1 = one_ring_v_ids1.len();
        let len2 = one_ring_v_ids2.len();

        let mut start_idx1 = orig_start_idx1;
        let mut start_idx2 = orig_start_idx2;

        let mut end_idx1 = (start_idx1 + 1) % len1;
        let mut end_idx2 = (start_idx2 + 1) % len2;

        let mut v_id1;
        let mut v_id2;

        while end_idx1 != orig_start_idx1 {
            v_id1 = one_ring_v_ids1[end_idx1];
            v_id2 = one_ring_v_ids2[end_idx2];

            #[cfg(feature = "rerun")]
            {
                self.log_vert_rerun("v_1", v_id1);
                self.log_vert_rerun("v_2", v_id2);
            }

            let mut shared = false;

            if common_v_ids.contains(&v_id1) {
                while v_id2 != v_id1 {
                    end_idx2 = (end_idx2 + 1) % len2;
                    v_id2 = one_ring_v_ids2[end_idx2];
                }

                shared = true;
            } else if common_v_ids.contains(&v_id2) {
                while v_id1 != v_id2 {
                    end_idx1 = (end_idx1 + 1) % len1;
                    v_id1 = one_ring_v_ids1[end_idx1];
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

                start_idx1 = end_idx1;
                start_idx2 = end_idx2;

                while common_v_ids.contains(&v_id1) {
                    start_idx1 = (start_idx1 + 1) % len1;
                    start_idx2 = (start_idx2 + 1) % len2;

                    v_id1 = one_ring_v_ids1[start_idx1];
                }

                end_idx1 = start_idx1;
                end_idx2 = start_idx2;

                start_idx1 = (end_idx1 - 1).rem_euclid(len1);
                start_idx2 = (end_idx2 - 1).rem_euclid(len2);

                if start_idx1 == orig_start_idx1 {
                    break;
                }

                tracing::info!("Index 1: Start {start_idx1}, End {end_idx1}");
            } else {
                end_idx1 = (end_idx1 + 1) % len1;
                end_idx2 = (end_idx2 + 1) % len2;
            }
        }

        let diff1 = (start_idx1 as i32 - end_idx1 as i32).unsigned_abs() as usize;
        let diff1 = diff1.min(len1 - diff1);

        let diff2 = (start_idx2 as i32 - end_idx2 as i32).unsigned_abs() as usize;
        let diff2 = diff2.min(len2 - diff2);

        if range_pairs_to_connect.is_empty() {
            range_pairs_to_connect.push(ConnectPair::new(
                orig_start_idx1..=(orig_start_idx1 + len1 - 1).rem_euclid(len1),
                len1,
                orig_start_idx2..=(orig_start_idx2 + len2 - 1).rem_euclid(len2),
                len2,
                !common_v_ids.is_empty(),
            ));
        } else if diff1 > 1 || diff2 > 1 || range_pairs_to_connect.is_empty() {
            range_pairs_to_connect.push(ConnectPair::new(
                start_idx1..=end_idx1,
                len1,
                start_idx2..=end_idx2,
                len2,
                !common_v_ids.is_empty(),
            ));
        }

        range_pairs_to_connect
    }

    fn find_start_indices(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        common_v_ids: &HashSet<VertexId>,
    ) -> Option<(usize, usize)> {
        if !common_v_ids.is_empty() {
            let mut v_id = *common_v_ids.iter().next().unwrap();

            let mut start_idx1 = one_ring_v_ids1.iter().position(|&v| v == v_id).unwrap();
            let mut start_idx2 = one_ring_v_ids2.iter().position(|&v| v == v_id).unwrap();

            let len1 = one_ring_v_ids1.len();
            let len2 = one_ring_v_ids2.len();

            while common_v_ids.contains(&v_id) {
                start_idx1 = (start_idx1 + 1) % len1;
                start_idx2 = (start_idx2 + 1) % len2;

                v_id = one_ring_v_ids1[start_idx1];
            }

            Some((
                (start_idx1 - 1).rem_euclid(len1),
                (start_idx2 - 1).rem_euclid(len2),
            ))
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
        if range1.end() < range1.start() {
            range1 = (*range1.start())..=(*range1.end() + len1);
        }
        if range2.end() < range2.start() {
            range2 = (*range2.start())..=(*range2.end() + len2);
        }

        ConnectPair {
            closed,
            ranges: [range1, range2],
        }
    }

    // This is a modified Bresenham algorithm usually used for drawing lines on a grid.
    fn compute_pairings(&self) -> Vec<Pairing> {
        let mut pairings = Vec::new();

        let mut range1 = self.ranges[0].clone();
        let mut range2 = self.ranges[1].clone();

        if self.closed {
            range1 = (*range1.start() + 1)..=(*range1.end() - 1);
            range2 = (*range2.start() + 1)..=(*range2.end() - 1);
        }

        let ranges = [range1.clone(), range2.clone()];

        let count1 = range1.count() as i32;
        let count2 = range2.count() as i32;

        let (single_range_idx, single_count, other_count) = if count1 > count2 {
            (1, count2, count1)
        } else {
            (0, count1, count2)
        };

        let single_range = ranges[single_range_idx].clone();
        let other_range = ranges[1 - single_range_idx].clone();

        let mut error = 2 * single_count - other_count;

        let mut single_idx_in_range = *single_range.start();
        let mut other_start = *other_range.start();
        let mut other_end = other_start;

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
        let mut faces = Vec::new();

        let mut others = self.other_range.clone();

        let mut prev_other_v_id = other_ids[others.next().unwrap() % other_ids.len()];

        for other_idx in others {
            let other_v_id = other_ids[other_idx % other_ids.len()];

            faces.push(PlannedFace::new(single_v_id, other_v_id, prev_other_v_id));

            prev_other_v_id = other_v_id;
        }

        faces
    }
}

#[derive(Debug)]
struct PlannedFace([VertexId; 3]);

impl PlannedFace {
    fn new(v_id1: VertexId, v_id2: VertexId, v_id3: VertexId) -> Self {
        PlannedFace([v_id1, v_id2, v_id3])
    }

    fn make_ccw(
        &mut self,
        vertex_normals: &SecondaryMap<VertexId, Vec3>,
        mesh_graph: &MeshGraph,
    ) -> Option<()> {
        let face_normal = vertex_normals[self.0[0]];

        let positions = self
            .0
            .iter()
            .filter_map(|&id| mesh_graph.positions.get(id).copied())
            .collect_array::<3>()?;

        let a = positions[1] - positions[0];
        let b = positions[2] - positions[0];
        let current_normal = a.cross(b);

        let (first, second) = if face_normal.dot(current_normal) < 0.0 {
            (self.0[1], self.0[0])
        } else {
            (self.0[0], self.0[1])
        };

        self.0[0] = first;
        self.0[1] = second;

        Some(())
    }

    fn insert_into_meshgraph(&self, mesh_graph: &mut MeshGraph) -> Option<CreateFace> {
        mesh_graph.create_face_from_vertices(self.0[0], self.0[1], self.0[2])
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        utils::{extend_with, get_tracing_subscriber},
        *,
    };
    use glam::*;

    #[test]
    fn test_vertex_merge_equal_count() {
        get_tracing_subscriber();

        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 1.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.5, 0.0);
        let p_3 = vec3(-1.0, -0.5, 0.0);
        let p_4 = vec3(0.0, -1.0, 0.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6 = vec3(1.0, 0.5, 0.0);

        let points = vec![p_c, p_1, p_2, p_3, p_4, p_5, p_6];
        let v_c_id = extend_with(&mut meshgraph, &points.clone(), Mat4::default(), 2.0, 1)[0];

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        // duplicates a mirrored version of the mesh above
        let mirror_mat = Mat4::from_rotation_translation(
            Quat::from_rotation_x(std::f32::consts::PI)
                .mul_quat(Quat::from_rotation_z(std::f32::consts::PI * 0.5)),
            vec3(0.0, 0.0, 3.0),
        );

        let v_c_m_id = extend_with(&mut meshgraph, &points, mirror_mat, 2.0, 1)[0];

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let result = meshgraph.merge_vertices_one_rings(v_c_id, v_c_m_id);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        assert_eq!(result.removed_faces.len(), 12);
        assert_eq!(result.removed_halfedges.len(), 24);
        assert_eq!(result.removed_vertices.len(), 2);

        assert_eq!(result.added_faces.len(), 12);
        assert_eq!(result.added_halfedges.len(), 24);
    }

    #[test]
    fn test_vertex_merge_different_count() {
        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 1.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.5, 0.0);
        let p_3 = vec3(-1.0, -0.5, 0.0);
        let p_4 = vec3(0.0, -1.0, 0.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6 = vec3(1.0, 0.5, 0.0);

        let v_c_id = extend_with(
            &mut meshgraph,
            &[p_c, p_1, p_2, p_3, p_4, p_5, p_6],
            Mat4::default(),
            2.0,
            1,
        )[0];

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        let mirror_mat = Mat4::from_rotation_translation(
            Quat::from_rotation_x(std::f32::consts::PI)
                .mul_quat(Quat::from_rotation_z(std::f32::consts::PI * 0.5)),
            vec3(0.0, 0.0, 3.0),
        );

        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.0, 0.0);
        let p_3 = vec3(-0.5, -1.0, 0.0);
        let p_4 = vec3(0.5, -1.0, 0.0);
        let p_5 = vec3(1.0, 0.0, 0.0);

        let v_c_m_id = extend_with(
            &mut meshgraph,
            &[p_c, p_1, p_2, p_3, p_4, p_5],
            mirror_mat,
            2.0,
            1,
        )[0];

        let result = meshgraph.merge_vertices_one_rings(v_c_id, v_c_m_id);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        assert_eq!(result.removed_faces.len(), 11);
        assert_eq!(result.removed_halfedges.len(), 22);
        assert_eq!(result.removed_vertices.len(), 2);

        assert_eq!(result.added_faces.len(), 11);
        assert_eq!(result.added_halfedges.len(), 22);
    }

    #[cfg(feature = "gltf")]
    #[test]
    fn test_vertex_merge_common_one_ring() {
        use crate::integrations::gltf;

        get_tracing_subscriber();

        let mut meshgraph = gltf::load("src/ops/cleanup/test/merge_common_one_ring.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let mut v_top_id = VertexId::default();
        let mut v_bottom_id = VertexId::default();

        for (v_id, pos) in &meshgraph.positions {
            if pos.x == 0.0 && pos.y == 0.0 {
                if pos.z > 0.0 {
                    v_top_id = v_id;
                } else {
                    v_bottom_id = v_id;
                }
            }
        }

        let result = meshgraph.merge_vertices_one_rings(v_top_id, v_bottom_id);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        assert_eq!(result.removed_faces.len(), 21);
        assert_eq!(result.removed_halfedges.len(), 46);
        assert_eq!(result.removed_vertices.len(), 3);

        assert_eq!(result.added_faces.len(), 13);
        assert_eq!(result.added_halfedges.len(), 22);
    }
}
