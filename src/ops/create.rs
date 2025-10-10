use glam::Vec3;
use tracing::instrument;

use crate::{error_none, FaceId, Halfedge, HalfedgeId, MeshGraph};

impl MeshGraph {
    pub fn create_face(&mut self, a: Vec3, b: Vec3, c: Vec3) -> FaceId {
        let a_id = self.insert_vertex(a);
        let b_id = self.insert_vertex(b);
        let c_id = self.insert_vertex(c);

        let he_a_id = self.insert_halfedge(b_id);
        let he_b_id = self.insert_halfedge(c_id);
        let he_c_id = self.insert_halfedge(a_id);

        let he_a_twin_id = self.insert_halfedge(a_id);
        let he_b_twin_id = self.insert_halfedge(b_id);
        let he_c_twin_id = self.insert_halfedge(c_id);

        self.halfedges[he_a_id].next = Some(he_b_id);
        self.halfedges[he_b_id].next = Some(he_c_id);
        self.halfedges[he_c_id].next = Some(he_a_id);

        self.halfedges[he_a_twin_id].next = Some(he_c_twin_id);
        self.halfedges[he_b_twin_id].next = Some(he_a_twin_id);
        self.halfedges[he_c_twin_id].next = Some(he_b_twin_id);

        self.halfedges[he_a_id].twin = Some(he_a_twin_id);
        self.halfedges[he_b_id].twin = Some(he_b_twin_id);
        self.halfedges[he_c_id].twin = Some(he_c_twin_id);

        self.halfedges[he_a_twin_id].twin = Some(he_a_id);
        self.halfedges[he_b_twin_id].twin = Some(he_b_id);
        self.halfedges[he_c_twin_id].twin = Some(he_c_id);

        let face_id = self.insert_face(he_a_id);

        self.halfedges[he_a_id].face = Some(face_id);
        self.halfedges[he_b_id].face = Some(face_id);
        self.halfedges[he_c_id].face = Some(face_id);

        self.vertices[a_id].outgoing_halfedge = Some(he_a_id);
        self.vertices[b_id].outgoing_halfedge = Some(he_b_id);
        self.vertices[c_id].outgoing_halfedge = Some(he_c_id);

        face_id
    }

    /// Returns `None` when an edge already has two faces
    pub fn add_face_to_edge(
        &mut self,
        he_id: HalfedgeId,
        opposite_vertex_pos: Vec3,
    ) -> Option<FaceId> {
        let mut he_a = self.boundary_he(he_id)?;
        let start_vertex_id_he_a = he_a.start_vertex(self)?;
        let he_a_id = self
            .vertices
            .get(start_vertex_id_he_a)
            .or_else(error_none!("Halfedge id not found"))?
            .outgoing_halfedge?;

        let c_id = self.insert_vertex(opposite_vertex_pos);

        let he_b_id = self.insert_halfedge(c_id);
        let he_c_id = self.insert_halfedge(start_vertex_id_he_a);

        let he_b_twin_id = self.insert_halfedge(he_a.end_vertex);
        let he_c_twin_id = self.insert_halfedge(c_id);

        he_a.next = Some(he_b_id);
        self.halfedges[he_b_id].next = Some(he_c_id);
        self.halfedges[he_c_id].next = Some(he_b_id);

        self.halfedges[he_b_twin_id].next = Some(he_a.twin?);
        self.halfedges[he_a.twin?].next = Some(he_c_twin_id);
        self.halfedges[he_c_twin_id].next = Some(he_b_twin_id);

        self.halfedges[he_b_id].twin = Some(he_b_twin_id);
        self.halfedges[he_c_id].twin = Some(he_c_twin_id);

        self.halfedges[he_b_twin_id].twin = Some(he_b_id);
        self.halfedges[he_c_twin_id].twin = Some(he_c_id);

        let face_id = self.insert_face(he_id);

        he_a.face = Some(face_id);

        face_id
    }

    #[instrument(skip(self))]
    pub fn fill_face(&mut self, he_id1: HalfedgeId, he_id2: HalfedgeId) -> Option<FaceId> {
        let he1 = self.boundary_he(he_id1)?;
        let he2 = self.boundary_he(he_id2)?;

        todo!()
    }

    #[instrument(skip(self))]
    pub fn boundary_he(&self, he_id: HalfedgeId) -> Option<Halfedge> {
        let mut he = *self
            .halfedges
            .get(he_id)
            .or_else(error_none!("Halfedge 1 not found"))?;

        if !he.is_boundary() {
            let twin_id = he.twin.or_else(error_none!("Twin missing"))?;

            he = *self
                .halfedges
                .get(twin_id)
                .or_else(error_none!("Twin not found"))?;
        }

        if he.is_boundary() {
            Some(he)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_create_face() {
        let mesh_graph = MeshGraph::new();
        let face_id: FaceId = todo!();

        let face = mesh_graph.faces[face_id];
    }
}
