use tracing::{error, instrument};

use crate::{MeshGraph, VertexId};

impl MeshGraph {
    /// Ensure that if one of the outgoing halfedges of this vertex is a boundary halfedge,
    /// then it will be the one in [`Vertex::outgoing_halfedge`](crate::Vertex::outgoing_halfedge).
    ///
    /// The method [`Vertex::is_boundary`](crate::Vertex::is_boundary) relies on this.
    #[instrument(skip(self))]
    pub fn make_outgoing_halfedge_boundary_if_possible(&mut self, vertex_id: VertexId) {
        if !self.vertices.contains_key(vertex_id) {
            error!("Vertex not found");
            return;
        }

        // A live boundary seed is kept as it is, so the ring order stays stable.
        if self.vertices[vertex_id]
            .outgoing_halfedge
            .and_then(|he_id| self.halfedges.get(he_id))
            .is_some_and(|he| he.is_boundary())
        {
            return;
        }

        // Scan the outgoing list rather than walking the ring from the current seed: the
        // walk rotates via `twin.next` and stops at the first boundary halfedge, so from an
        // interior seed it never reaches the boundary outgoing halfedge it is looking for.
        let boundary_he_id = self.outgoing_halfedges.get(vertex_id).and_then(|out| {
            out.iter().copied().find(|he_id| {
                self.halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.is_boundary())
            })
        });

        if let Some(boundary_he_id) = boundary_he_id {
            self.vertices[vertex_id].outgoing_halfedge = Some(boundary_he_id);
        }
    }

    /// Ensure that if one of the outgoing halfedges of any vertex is a boundary halfedge,
    /// then it will be the one in [`Vertex::outgoing_halfedge`](crate::Vertex::outgoing_halfedge).
    ///
    /// The method [`Vertex::is_boundary`](crate::Vertex::is_boundary) relies on this.
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
