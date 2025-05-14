use hashbrown::HashSet;

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId};

impl MeshGraph {
    /// Deletes a face from the mesh graph.
    ///
    /// It also deletes the vertices and halfedges that are no
    /// longer connected to any other faces.
    ///
    /// Returns the ids of the removed vertices and halfedges.
    pub fn delete_face(&mut self, face_id: FaceId) -> (Vec<VertexId>, Vec<HalfedgeId>) {
        let face = self.faces[face_id];

        let vertices = face.vertices(self);
        let halfedges = face.halfedges(self);

        let mut deleted_halfedges = HashSet::with_capacity(4);

        for he_id in halfedges {
            let mut halfedge = self.halfedges[he_id];
            let twin_id = halfedge.twin();

            if self.halfedges[twin_id].is_boundary() {
                deleted_halfedges.insert(he_id);
                deleted_halfedges.insert(twin_id);
            } else {
                halfedge.face = None;
            }
        }

        let mut deleted_vertices = Vec::with_capacity(3);

        for &vertex_id in &vertices {
            let mut all_deleted = true;
            for he_id in self.vertices[vertex_id].outgoing_halfedges(self) {
                if !deleted_halfedges.contains(&he_id) {
                    all_deleted = false;
                    break;
                }
            }

            if all_deleted {
                self.positions.remove(vertex_id);
                if let Some(normals) = &mut self.vertex_normals {
                    normals.remove(vertex_id);
                }
                self.vertices.remove(vertex_id);

                deleted_vertices.push(vertex_id);
            }
        }

        for &he_id in &deleted_halfedges {
            self.halfedges.remove(he_id);
        }

        self.faces.remove(face_id);

        (deleted_vertices, Vec::from_iter(deleted_halfedges))
    }
}
