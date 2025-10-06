use glam::Vec3;
use tracing::instrument;

use crate::{FaceId, Halfedge, HalfedgeId, MeshGraph, error_none};

impl MeshGraph {
    pub fn create_face(&mut self, a: Vec3, b: Vec3, c: Vec3) -> FaceId {
        todo!()
    }

    /// Returns `None` when an edge already has two faces
    pub fn add_face_to_edge(
        &mut self,
        he_id: HalfedgeId,
        opposite_vertex_pos: Vec3,
    ) -> Option<FaceId> {
        todo!()
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

        Some(he)
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
