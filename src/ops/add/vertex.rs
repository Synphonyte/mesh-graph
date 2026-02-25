use glam::Vec3;

use crate::{MeshGraph, Vertex, VertexId};

impl MeshGraph {
    /// Inserts a vertex and it's position into the mesh graph.
    /// It doesn't do any connections.
    pub fn add_vertex(&mut self, position: Vec3) -> VertexId {
        let vertex = Vertex::default();
        let vertex_id = self.vertices.insert(vertex);
        self.positions.insert(vertex_id, position);

        self.outgoing_halfedges.insert(vertex_id, vec![]);

        vertex_id
    }
}
