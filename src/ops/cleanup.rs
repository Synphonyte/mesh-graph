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

    #[inline]
    #[instrument(skip(self))]
    pub fn make_outgoing_halfedge_boundary_if_possible(&mut self, vertex_id: VertexId) {
        let _ = self.make_outgoing_halfedge_boundary_if_possible_inner(vertex_id);
    }

    fn make_outgoing_halfedge_boundary_if_possible_inner(
        &mut self,
        vertex_id: VertexId,
    ) -> Option<()> {
        let vertex = self
            .vertices
            .get(vertex_id)
            .or_else(error_none!("Vertex not found"))?;

        let he = self
            .halfedges
            .get(
                vertex
                    .outgoing_halfedge
                    .or_else(error_none!("Vertex should have a outgoing halfedge"))?,
            )
            .or_else(error_none!("Halfedge not found"))?;

        let face = self
            .faces
            .get(
                he.face
                    .or_else(error_none!("Face should be already specified"))?,
            )
            .or_else(error_none!("Face not found"))?;

        let boundary_he_id = face.halfedges(self).find_map(|he_id| {
            let he = self
                .halfedges
                .get(he_id)
                .or_else(error_none!("Halfedge not found"))?;
            if he.is_boundary() {
                Some(he_id)
            } else {
                None
            }
        });

        if boundary_he_id.is_some() {
            self.vertices
                .get_mut(vertex_id)
                .or_else(error_none!("Vertex not found"))?
                .outgoing_halfedge = boundary_he_id;
        }

        Some(())
    }
}
