mod edge_boundary;
mod vertex_neighborhood;

use tracing::instrument;

use crate::{error_none, FaceId, HalfedgeId, MeshGraph, VertexId};

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
    use crate::{
        utils::{extend_with, get_tracing_subscriber},
        *,
    };
    use glam::*;
    use itertools::Itertools;

    fn reconnect_vertex(
        meshgraph: &mut MeshGraph,
        old_vertex_id: VertexId,
        new_vertex_id: VertexId,
    ) {
        tracing::info!(
            "Reconnecting vertex {:?} to {:?}",
            old_vertex_id,
            new_vertex_id
        );
        let incoming_halfedges = meshgraph.vertices[old_vertex_id]
            .incoming_halfedges(meshgraph)
            .collect_vec();

        for he_in_id in incoming_halfedges {
            meshgraph.halfedges[he_in_id].end_vertex = new_vertex_id;
        }

        let outcoming_halfedges = meshgraph.vertices[old_vertex_id]
            .outgoing_halfedges(meshgraph)
            .collect_vec();

        meshgraph.outgoing_halfedges[new_vertex_id].extend(outcoming_halfedges);

        meshgraph.delete_only_vertex(old_vertex_id);
    }

    // #[test]
    fn test_vertex_join() {
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
        extend_with(&mut meshgraph, &points.clone(), Mat4::default(), 5.0, 3);

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

        extend_with(&mut meshgraph, &points, mirror_mat, 3.0, 2);

        // TODO: Call join function on meshgraph

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }
    }

    // #[test]
    fn test_vertex_join_non_similar_vertex_count() {
        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 1.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.5, 0.0);
        let p_3 = vec3(-1.0, -0.5, 0.0);
        let p_4 = vec3(0.0, -1.0, 0.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6 = vec3(1.0, 0.5, 0.0);

        extend_with(
            &mut meshgraph,
            &[p_c, p_1, p_2, p_3, p_4, p_5, p_6],
            Mat4::default(),
            3.0,
            3,
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

        extend_with(
            &mut meshgraph,
            &[p_c, p_1, p_2, p_3, p_4, p_5],
            mirror_mat,
            3.0,
            1,
        );

        // TODO: Call join function on meshgraph
        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }
    }

    #[test]
    fn test_vertex_join_same_edge_and_same_vertex() {
        get_tracing_subscriber();
        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 1.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2u = vec3(-1.0, 0.5, 2.0);
        let p_3u = vec3(-1.0, -0.5, 2.0);
        let p_4u = vec3(0.0, -1.0, 2.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6u = vec3(1.0, 0.5, 2.0);

        let vertex_ids = extend_with(
            &mut meshgraph,
            &[p_c, p_1, p_2u, p_3u, p_4u, p_5, p_6u],
            Mat4::default(),
            1.0,
            0,
        );

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        let p_m_c = vec3(0.0, 0.0, 4.0);
        let p_m_1 = vec3(0.0, 1.0, 4.0);
        let p_m_5 = vec3(1.0, -0.5, 4.0);

        let v_m_c_id = meshgraph.insert_vertex(p_m_c);
        let v_m_1_id = meshgraph.insert_vertex(p_m_1);
        let v_m_5_id = meshgraph.insert_vertex(p_m_5);

        let (h_c_m_1_id, _) = meshgraph.insert_or_get_edge(v_m_c_id, v_m_1_id);
        let (h_c_2_id, h_2_c_id) = meshgraph.insert_or_get_edge(v_m_c_id, vertex_ids[2]);
        let (h_c_3_id, h_3_c_id) = meshgraph.insert_or_get_edge(v_m_c_id, vertex_ids[3]);
        let (h_c_4_id, h_4_c_id) = meshgraph.insert_or_get_edge(v_m_c_id, vertex_ids[4]);
        let (h_c_m_5_id, _) = meshgraph.insert_or_get_edge(v_m_c_id, v_m_5_id);
        let (h_c_6_id, _) = meshgraph.insert_or_get_edge(v_m_c_id, vertex_ids[6]);

        meshgraph
            .create_face_from_halfedges(h_c_m_1_id, h_2_c_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_2_id, h_3_c_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_3_id, h_4_c_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_4_id, h_c_m_5_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_m_5_id, h_c_6_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_6_id, h_c_m_1_id)
            .unwrap();

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        // merge a pair of vertices together
        // let v_6_id = vertex_ids[6];
        // let v_m_5_id = mirrored_vertex_ids[5];

        // reconnect_vertex(&mut meshgraph, v_m_5_id, v_6_id);

        // TODO: Call join function on meshgraph
    }
}
