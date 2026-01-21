mod edge_boundary;
mod merge;
mod vertex_neighborhood;

use tracing::instrument;

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

impl MeshGraph {
    /// Test if two faces have at least one halfedge in common.
    #[inline]
    #[instrument(skip(self))]
    pub fn faces_share_edge(&self, face_id1: FaceId, face_id2: FaceId) -> bool {
        self.faces_share_edge_inner(face_id1, face_id2).is_some()
    }

    fn faces_share_edge_inner(&self, face_id1: FaceId, face_id2: FaceId) -> Option<()> {
        let face1 = self
            .faces
            .get(face_id1)
            .or_else(error_none!("Face 1 not found"))?;
        let face2 = self
            .faces
            .get(face_id2)
            .or_else(error_none!("Face 2 not found"))?;

        for edge_id1 in face1.halfedges(self) {
            let edge1 = self
                .halfedges
                .get(edge_id1)
                .or_else(error_none!("Halfedge 1 not found"))?;

            let start_pos1 = self
                .positions
                .get(edge1.start_vertex(self)?)
                .or_else(error_none!("Position of start vertex 1 not found"))?;
            let end_pos1 = self
                .positions
                .get(edge1.end_vertex)
                .or_else(error_none!("Position of end vertex 1 not found"))?;

            for edge_id2 in face2.halfedges(self) {
                let edge2 = self
                    .halfedges
                    .get(edge_id2)
                    .or_else(error_none!("Halfedge 2 not found"))?;

                let start_pos2 = self
                    .positions
                    .get(edge2.start_vertex(self)?)
                    .or_else(error_none!("Position of start vertex 2 not found"))?;
                let end_pos2 = self
                    .positions
                    .get(edge2.end_vertex)
                    .or_else(error_none!("Position of end vertex 2 not found"))?;

                if start_pos1 == start_pos2 && end_pos1 == end_pos2 {
                    return Some(());
                }
            }
        }

        None
    }

    /// Test if two faces share all vertices.
    #[inline]
    #[instrument(skip(self))]
    pub fn faces_share_all_vertices(&self, face_id1: FaceId, face_id2: FaceId) -> bool {
        self.faces_share_all_vertices_inner(face_id1, face_id2)
            .is_some()
    }

    fn faces_share_all_vertices_inner(&self, face_id1: FaceId, face_id2: FaceId) -> Option<()> {
        let face1 = self
            .faces
            .get(face_id1)
            .or_else(error_none!("Face 1 not found"))?;
        let face2 = self
            .faces
            .get(face_id2)
            .or_else(error_none!("Face 2 not found"))?;

        'outer: for vertex_id1 in face1.vertices(self) {
            let pos1 = self
                .positions
                .get(vertex_id1)
                .or_else(error_none!("Position of vertex 1 not found"))?;

            for vertex_id2 in face2.vertices(self) {
                let pos2 = self
                    .positions
                    .get(vertex_id2)
                    .or_else(error_none!("Position of vertex 2 not found"))?;

                if pos1 == pos2 {
                    continue 'outer;
                }
            }

            return None;
        }

        Some(())
    }

    /// Test if two halfedges share all vertices.
    #[inline]
    #[instrument(skip(self))]
    pub fn halfedges_share_all_vertices(
        &self,
        halfedge_id1: HalfedgeId,
        halfedge_id2: HalfedgeId,
    ) -> bool {
        self.halfedges_share_all_vertices_inner(halfedge_id1, halfedge_id2)
            .is_some()
    }

    fn halfedges_share_all_vertices_inner(
        &self,
        halfedge_id1: HalfedgeId,
        halfedge_id2: HalfedgeId,
    ) -> Option<()> {
        let he1 = self
            .halfedges
            .get(halfedge_id1)
            .or_else(error_none!("Halfedge 1 not found"))?;
        let he2 = self
            .halfedges
            .get(halfedge_id2)
            .or_else(error_none!("Halfedge 2 not found"))?;

        let start_id1 = he1.start_vertex(self)?;

        let start_id2 = he2.start_vertex(self)?;

        (start_id1 == start_id2 && he1.end_vertex == he2.end_vertex).then_some(())
    }

    /// Test if two vertices have the exact same position.
    #[inline]
    #[instrument(skip(self))]
    pub fn vertices_share_position(&self, vertex_id1: VertexId, vertex_id2: VertexId) -> bool {
        self.vertices_share_position_inner(vertex_id1, vertex_id2)
            .is_some()
    }

    fn vertices_share_position_inner(
        &self,
        vertex_id1: VertexId,
        vertex_id2: VertexId,
    ) -> Option<()> {
        (self
            .positions
            .get(vertex_id1)
            .or_else(error_none!("Position of vertex 1 not found"))?
            == self
                .positions
                .get(vertex_id2)
                .or_else(error_none!("Position of vertex 2 not found"))?)
        .then_some(())
    }
}

#[cfg(test)]
mod tests {
    use crate::{utils::get_tracing_subscriber, *};
    use glam::*;

    fn extend_outer_corners(
        meshgraph: &mut MeshGraph,
        new_vertex_ids: &mut Vec<VertexId>,
        outer_vertex_ids: &[VertexId],
        scalar: f32,
        steps: usize,
    ) {
        if steps == 0 {
            return;
        }

        let mut corner_vertex_ids = Vec::with_capacity(outer_vertex_ids.len());

        // mesh star corners to make the mesh larger
        for i in 0..outer_vertex_ids.len() {
            let point_1 = meshgraph.positions.get(outer_vertex_ids[i]).unwrap();
            let point_2 = meshgraph
                .positions
                .get(outer_vertex_ids[(i + 1) % outer_vertex_ids.len()])
                .unwrap();

            let point_3 = point_1
                + ((point_2 - point_1) * 0.5)
                + (point_1 + point_2).normalize() * scalar / steps as f32;
            let vertex_id = meshgraph.insert_vertex(point_3);
            corner_vertex_ids.push(vertex_id);
        }

        for cv_i in 0..corner_vertex_ids.len() {
            let corner_vertex_id = corner_vertex_ids[cv_i];
            let vertex_id = outer_vertex_ids[cv_i];
            let next_vertext_id = outer_vertex_ids[(cv_i + 1) % outer_vertex_ids.len()];
            let halfedge_vertex_to_corner_id = meshgraph
                .insert_or_get_edge(vertex_id, corner_vertex_id)
                .start_to_end_he_id;
            let halfedge_vertex_to_next_vertex_id = meshgraph
                .insert_or_get_edge(vertex_id, next_vertext_id)
                .start_to_end_he_id;

            meshgraph
                .create_face_from_halfedges(
                    halfedge_vertex_to_corner_id,
                    halfedge_vertex_to_next_vertex_id,
                )
                .unwrap();

            let halfedge_corner_to_next_vertex_id = meshgraph
                .insert_or_get_edge(corner_vertex_id, next_vertext_id)
                .start_to_end_he_id;

            let halfedge_next_vertex_to_next_corner_vertex_id = meshgraph
                .insert_or_get_edge(
                    next_vertext_id,
                    corner_vertex_ids[(cv_i + 1) % corner_vertex_ids.len()],
                )
                .start_to_end_he_id;

            meshgraph
                .create_face_from_halfedges(
                    halfedge_corner_to_next_vertex_id,
                    halfedge_next_vertex_to_next_corner_vertex_id,
                )
                .unwrap();
        }

        extend_outer_corners(
            meshgraph,
            new_vertex_ids,
            &corner_vertex_ids,
            scalar,
            steps - 1,
        );

        new_vertex_ids.extend(corner_vertex_ids);
    }

    /// Extend a mesh graph with new points.
    /// Expects the first point to be the geometrical center of the new vertices.
    /// Mesh then extends further by `steps` iterations from the center outward.
    fn extend_with(
        meshgraph: &mut MeshGraph,
        center_and_points: &[Vec3],
        matrix: Mat4,
        scalar: f32,
        steps: usize,
    ) -> VertexId {
        let (center, points) = center_and_points.split_first().unwrap();
        let center_id = meshgraph.insert_vertex(*center);

        let mut vertex_ids = Vec::new();
        let mut halfedge_ids = Vec::new();

        for point in points {
            let vertex_id = meshgraph.insert_vertex(*point);
            let halfedge_id = meshgraph
                .insert_or_get_edge(center_id, vertex_id)
                .start_to_end_he_id;

            vertex_ids.push(vertex_id);
            halfedge_ids.push(halfedge_id);
        }

        for i in 0..points.len() {
            meshgraph
                .create_face_from_halfedges(halfedge_ids[i], halfedge_ids[(i + 1) % points.len()])
                .unwrap();
        }

        let mut new_vertex_ids = vertex_ids.clone();
        extend_outer_corners(meshgraph, &mut new_vertex_ids, &vertex_ids, scalar, steps);
        new_vertex_ids.push(center_id);

        for new_vertex_id in new_vertex_ids {
            if let Some(pos) = meshgraph.positions.get_mut(new_vertex_id) {
                *pos = matrix.project_point3(*pos);
            };
        }

        center_id
    }

    #[test]
    fn test_vertex_join_equal_count() {
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
        let v_c_id = extend_with(&mut meshgraph, &points.clone(), Mat4::default(), 2.0, 1);

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

        let v_c_m_id = extend_with(&mut meshgraph, &points, mirror_mat, 2.0, 1);

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

        assert_eq!(result.added_faces.len(), 24);
        assert_eq!(result.added_halfedges.len(), 24);
    }

    #[test]
    fn test_vertex_join_different_count() {
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
        );

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
        );

        let result = meshgraph.merge_vertices_one_rings(v_c_id, v_c_m_id);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        assert_eq!(result.removed_faces.len(), 12);
        assert_eq!(result.removed_halfedges.len(), 24);
        assert_eq!(result.removed_vertices.len(), 2);

        assert_eq!(result.added_faces.len(), 24);
        assert_eq!(result.added_halfedges.len(), 24);
    }
}
