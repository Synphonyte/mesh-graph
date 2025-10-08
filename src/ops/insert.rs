use glam::Vec3;

use crate::{Face, FaceId, Halfedge, HalfedgeId, MeshGraph, Vertex, VertexId};

impl MeshGraph {
    /// Inserts a vertex and it's position into the mesh graph.
    /// It doesn't do any connections.
    pub fn insert_vertex(&mut self, position: Vec3) -> VertexId {
        let vertex = Vertex::default();
        let vertex_id = self.vertices.insert(vertex);
        self.positions.insert(vertex_id, position);

        vertex_id
    }

    /// Inserts a halfedge into the mesh graph. It only connects the halfedge to the given end vertex but not the reverse.
    /// It also doesn't do any other connections.
    pub fn insert_halfedge(&mut self, end_vertex: VertexId) -> HalfedgeId {
        let halfedge = Halfedge {
            end_vertex,
            next: None,
            twin: None,
            face: None,
        };
        self.halfedges.insert(halfedge)
    }

    /// Inserts a face into the mesh graph. It only connects the face to the given halfedge but not the reverse.
    /// It also doesn't do any other connections.
    pub fn insert_face(&mut self, halfedge: HalfedgeId) -> FaceId {
        let face_id = self.faces.insert_with_key(|id| Face {
            halfedge,
            index: self.next_index,
            id,
        });

        self.index_to_face_id.insert(self.next_index, face_id);

        self.next_index += 1;

        face_id
    }
}
