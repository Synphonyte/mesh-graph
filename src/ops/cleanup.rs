use crate::{FaceId, HalfedgeId, MeshGraph, VertexId};

impl MeshGraph {
    /// Test if two faces have at least one halfedge in common.
    pub fn faces_share_edge(&self, face_id1: FaceId, face_id2: FaceId) -> bool {
        let face1 = self.faces[face_id1];
        let face2 = self.faces[face_id2];

        for edge_id1 in face1.halfedges(self) {
            let edge1 = self.halfedges[edge_id1];

            let start1 = self.positions[edge1.start_vertex(self)];
            let end1 = self.positions[edge1.end_vertex];

            for edge_id2 in face2.halfedges(self) {
                let edge2 = self.halfedges[edge_id2];

                let start2 = self.positions[edge2.start_vertex(self)];
                let end2 = self.positions[edge2.end_vertex];

                if start1 == start2 && end1 == end2 {
                    return true;
                }
            }
        }

        false
    }

    /// Test if two faces share all vertices.
    pub fn faces_share_all_vertices(&self, face_id1: FaceId, face_id2: FaceId) -> bool {
        let face1 = self.faces[face_id1];
        let face2 = self.faces[face_id2];

        let face2_vertices = face2.vertices(self);

        'outer: for vertex_id1 in face1.vertices(self) {
            let pos1 = self.positions[vertex_id1];

            for &vertex_id2 in &face2_vertices {
                let pos2 = self.positions[vertex_id2];

                if pos1 == pos2 {
                    continue 'outer;
                }
            }

            return false;
        }

        true
    }

    /// Test if two halfedges share all vertices.
    pub fn halfedges_share_all_vertices(
        &self,
        halfedge_id1: HalfedgeId,
        halfedge_id2: HalfedgeId,
    ) -> bool {
        let edge1 = self.halfedges[halfedge_id1];
        let edge2 = self.halfedges[halfedge_id2];

        let start1 = self.positions[edge1.start_vertex(self)];
        let end1 = self.positions[edge1.end_vertex];

        let start2 = self.positions[edge2.start_vertex(self)];
        let end2 = self.positions[edge2.end_vertex];

        start1 == start2 && end1 == end2
    }

    /// Test if two vertices have the exact same position.
    pub fn vertices_share_position(&self, vertex_id1: VertexId, vertex_id2: VertexId) -> bool {
        self.positions[vertex_id1] == self.positions[vertex_id2]
    }

    pub fn make_outgoing_halfedge_boundary_if_possible(&mut self, _vertex_id: VertexId) {
        todo!()
    }
}
