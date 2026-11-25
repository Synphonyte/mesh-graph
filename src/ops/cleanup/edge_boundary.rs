use tracing::{error, instrument};

use slotmap::SlotMap;

use crate::ops::remove::push_unique;
use crate::{Halfedge, HalfedgeId, MeshGraph, VertexId};

/// See [`MeshGraph::boundary_outgoing_halfedge`]. Takes the parts it reads, so
/// `rebuild_outgoing_halfedges` can call it while it holds `vertices` mutably.
pub(crate) fn boundary_outgoing_halfedge(
    halfedges: &SlotMap<HalfedgeId, Halfedge>,
    outgoing: Option<&[HalfedgeId]>,
    seed: Option<HalfedgeId>,
) -> Option<HalfedgeId> {
    let is_boundary = |he_id: &HalfedgeId| halfedges.get(*he_id).is_some_and(|he| he.is_boundary());
    seed.filter(is_boundary)
        .or_else(|| outgoing?.iter().copied().find(is_boundary))
}

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

        if let Some(boundary_he_id) = self.boundary_outgoing_halfedge(vertex_id) {
            self.vertices[vertex_id].outgoing_halfedge = Some(boundary_he_id);
        }
    }

    /// The boundary outgoing halfedge a boundary vertex should be seeded with, or `None`
    /// for an interior vertex.
    ///
    /// A live boundary seed is returned as it is, so the ring order stays stable.
    /// Otherwise the vertex's outgoing list is scanned rather than walking the ring from the
    /// seed: the walk rotates via `twin.next` and stops at the first boundary halfedge, so
    /// from an interior seed it never reaches the boundary outgoing halfedge.
    pub(crate) fn boundary_outgoing_halfedge(&self, vertex_id: VertexId) -> Option<HalfedgeId> {
        boundary_outgoing_halfedge(
            &self.halfedges,
            self.outgoing_halfedges.get(vertex_id).map(Vec::as_slice),
            self.vertices.get(vertex_id)?.outgoing_halfedge,
        )
    }

    /// Calls [`Self::make_outgoing_halfedge_boundary_if_possible`] on each live vertex of
    /// `centers` and on each vertex of their one-rings.
    pub(crate) fn seed_boundary_around(&mut self, centers: impl IntoIterator<Item = VertexId>) {
        let mut vertices: Vec<VertexId> = Vec::new();
        for center in centers {
            let Some(out) = self.outgoing_halfedges.get(center) else {
                continue;
            };
            push_unique(&mut vertices, center);
            for he_id in out {
                if let Some(he) = self.halfedges.get(*he_id) {
                    push_unique(&mut vertices, he.end_vertex);
                }
            }
        }
        for v_id in vertices {
            if self.vertices.contains_key(v_id) {
                self.make_outgoing_halfedge_boundary_if_possible(v_id);
            }
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
