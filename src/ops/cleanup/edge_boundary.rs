use tracing::{error, instrument};

use crate::{MeshGraph, VertexId, error_none};

impl MeshGraph {
    /// Ensure that if one of the outgoing halfedges of this vertex is a boundary halfedge,
    /// then it will be the one in [`Vertex::outgoing_halfedge`].
    ///
    /// The method [`Vertex::is_boundary`] relies on this.
    #[instrument(skip(self))]
    pub fn make_outgoing_halfedge_boundary_if_possible(&mut self, vertex_id: VertexId) {
        if !self.vertices.contains_key(vertex_id) {
            error!("Vertex not found");
            return;
        }

        let boundary_he_id = self.vertices[vertex_id]
            .outgoing_halfedges(self)
            .filter_map(|he_id| {
                self.halfedges
                    .get(he_id)
                    .or_else(error_none!("Halfedge not found"))
                    .map(|he| (he, he_id))
            })
            .find(|(he, _)| he.is_boundary());

        if let Some((_, boundary_he_id)) = boundary_he_id {
            self.vertices[vertex_id].outgoing_halfedge = Some(boundary_he_id);
        }
    }

    /// Ensure that if one of the outgoing halfedges of any vertex is a boundary halfedge,
    /// then it will be the one in [`Vertex::outgoing_halfedge`].
    ///
    /// The method [`Vertex::is_boundary`] relies on this.
    #[instrument(skip(self))]
    pub fn make_all_outgoing_halfedges_boundary_if_possible(&mut self) {
        let boundary_pairs = self
            .halfedges
            .iter()
            .filter_map(|(he_id, he)| {
                if he.is_boundary() {
                    he.start_vertex(self).map(|v_id| (v_id, he_id))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();

        for (boundary_vertex_id, boundary_he_id) in boundary_pairs {
            if let Some(vertex) = self.vertices.get_mut(boundary_vertex_id) {
                vertex.outgoing_halfedge = Some(boundary_he_id);
            } else {
                error!("Vertex not found");
            }
        }
    }
}
