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
        let edge1 = self
            .halfedges
            .get(halfedge_id1)
            .or_else(error_none!("Halfedge 1 not found"))?;
        let edge2 = self
            .halfedges
            .get(halfedge_id2)
            .or_else(error_none!("Halfedge 2 not found"))?;

        let start1 = self
            .positions
            .get(edge1.start_vertex(self)?)
            .or_else(error_none!("Position of start vertex 1 not found"))?;
        let end1 = self
            .positions
            .get(edge1.end_vertex)
            .or_else(error_none!("Position of end vertex 1 not found"))?;

        let start2 = self
            .positions
            .get(edge2.start_vertex(self)?)
            .or_else(error_none!("Position of start vertex 2 not found"))?;
        let end2 = self
            .positions
            .get(edge2.end_vertex)
            .or_else(error_none!("Position of end vertex 2 not found"))?;

        (start1 == start2 && end1 == end2).then_some(())
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
    use crate::*;
    use glam::*;

    #[test]
    fn test_vertex_join() {
        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 1.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.5, 0.0);
        let p_3 = vec3(-1.0, -0.5, 0.0);
        let p_4 = vec3(0.0, -1.0, 0.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6 = vec3(1.0, 0.5, 0.0);

        let v_c_id = meshgraph.insert_vertex(p_c);
        let v_1_id = meshgraph.insert_vertex(p_1);
        let v_2_id = meshgraph.insert_vertex(p_2);
        let v_3_id = meshgraph.insert_vertex(p_3);
        let v_4_id = meshgraph.insert_vertex(p_4);
        let v_5_id = meshgraph.insert_vertex(p_5);
        let v_6_id = meshgraph.insert_vertex(p_6);

        let (h_c_v1_id, _) = meshgraph.insert_or_get_edge(v_c_id, v_1_id);
        let (h_c_v2_id, _) = meshgraph.insert_or_get_edge(v_c_id, v_2_id);
        let (h_c_v3_id, _) = meshgraph.insert_or_get_edge(v_c_id, v_3_id);
        let (h_c_v4_id, _) = meshgraph.insert_or_get_edge(v_c_id, v_4_id);
        let (h_c_v5_id, _) = meshgraph.insert_or_get_edge(v_c_id, v_5_id);
        let (h_c_v6_id, _) = meshgraph.insert_or_get_edge(v_c_id, v_6_id);

        meshgraph
            .create_face_from_halfedges(h_c_v1_id, h_c_v2_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_v2_id, h_c_v3_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_v3_id, h_c_v4_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_v4_id, h_c_v5_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_v5_id, h_c_v6_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(h_c_v6_id, h_c_v1_id)
            .unwrap();

        let mirror_mat = Mat4::from_rotation_translation(
            Quat::from_rotation_x(std::f32::consts::PI),
            vec3(0.0, 0.0, 3.0),
        );
        let v_c_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_c));
        let v_1_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_1));
        let v_2_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_2));
        let v_3_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_3));
        let v_4_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_4));
        let v_5_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_5));
        let v_6_m_id = meshgraph.insert_vertex(mirror_mat.project_point3(p_6));
        let (h_c_v1_m_id, _) = meshgraph.insert_or_get_edge(v_c_m_id, v_1_m_id);
        let (h_c_v2_m_id, _) = meshgraph.insert_or_get_edge(v_c_m_id, v_2_m_id);
        let (h_c_v3_m_id, _) = meshgraph.insert_or_get_edge(v_c_m_id, v_3_m_id);
        let (h_c_v4_m_id, _) = meshgraph.insert_or_get_edge(v_c_m_id, v_4_m_id);
        let (h_c_v5_m_id, _) = meshgraph.insert_or_get_edge(v_c_m_id, v_5_m_id);
        let (h_c_v6_m_id, _) = meshgraph.insert_or_get_edge(v_c_m_id, v_6_m_id);

        meshgraph
            .create_face_from_halfedges(h_c_v1_m_id, h_c_v2_m_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_v2_m_id, h_c_v3_m_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_v3_m_id, h_c_v4_m_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_v4_m_id, h_c_v5_m_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_v5_m_id, h_c_v6_m_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(h_c_v6_m_id, h_c_v1_m_id)
            .unwrap();

        // TODO: Call join function on meshgraph

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }
    }
}
