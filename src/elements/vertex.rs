use tracing::{error, instrument};

use crate::{CircularHalfedgesIterator, MeshGraph, error_none, utils::unwrap_or_return};

use super::{FaceId, HalfedgeId, VertexId};

/// A vertex is an corner point of a face.
///
/// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/all.svg" alt="Connectivity" style="max-width: 50em" />
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vertex {
    /// One of the halfedges with this vertex as start point.
    /// If possible this is a boundary halfedge, i.e. it has no associated face.
    ///
    /// After the mesh graph is constructed correctly, this is always `Some`.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/outgoing_halfedge.svg" alt="Connectivity" style="max-width: 50em" />
    pub outgoing_halfedge: Option<HalfedgeId>,
}

impl Vertex {
    /// One of the incoming halfedges of this vertex.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/incoming_halfedge.svg" alt="Connectivity" style="max-width: 50em" />
    #[instrument(skip(mesh_graph))]
    pub fn incoming_halfedge(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        mesh_graph
            .halfedges
            .get(
                self.outgoing_halfedge
                    .or_else(error_none!("Vertex has no outgoing halfedge"))?,
            )
            .or_else(error_none!("Outgoing halfedge not found"))
            .and_then(|halfedge| halfedge.twin.or_else(error_none!("Twin is None")))
    }

    /// Returns all halfedges that point away from this vertex.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/outgoing_halfedges.svg" alt="Connectivity" style="max-width: 50em" />
    #[instrument(skip(mesh_graph))]
    pub fn outgoing_halfedges<'a>(
        &self,
        mesh_graph: &'a MeshGraph,
    ) -> CircularHalfedgesIterator<'a> {
        CircularHalfedgesIterator::new(
            self.outgoing_halfedge,
            mesh_graph,
            |he, mesh_graph| {
                mesh_graph
                    .halfedges
                    .get(he)
                    .or_else(error_none!("Halfedge is None"))?
                    .cw_rotated_neighbour(mesh_graph)
            },
            100_000,
        )
    }

    /// Returns all halfedges that point towards this vertex
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/incoming_halfedges.svg" alt="Connectivity" style="max-width: 50em" />
    #[instrument(skip(mesh_graph))]
    pub fn incoming_halfedges(&self, mesh_graph: &MeshGraph) -> impl Iterator<Item = HalfedgeId> {
        self.outgoing_halfedges(mesh_graph).filter_map(|he_id| {
            mesh_graph
                .halfedges
                .get(he_id)
                .or_else(error_none!("Halfedge is None"))?
                .twin
                .or_else(error_none!("Twin is None"))
        })
    }

    /// Returns all faces incident to this vertex.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/faces.svg" alt="Connectivity" style="max-width: 50em" />
    #[instrument(skip(mesh_graph))]
    pub fn faces(&self, mesh_graph: &MeshGraph) -> impl Iterator<Item = FaceId> {
        self.outgoing_halfedges(mesh_graph).filter_map(|he| {
            mesh_graph
                .halfedges
                .get(he)
                .or_else(error_none!("Halfedge is None"))?
                .face
        })
    }

    /// Returns all neighbouring (connected through an edge) vertices of this vertex.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/neighbours.svg" alt="Connectivity" style="max-width: 50em" />
    #[instrument(skip(mesh_graph))]
    pub fn neighbours(&self, mesh_graph: &MeshGraph) -> impl Iterator<Item = VertexId> {
        self.outgoing_halfedges(mesh_graph).filter_map(|he| {
            mesh_graph
                .halfedges
                .get(he)
                .or_else(error_none!("Halfedge is None"))
                .map(|he| he.end_vertex)
        })
    }

    /// The degree of this vertex, i.e., the number of edges incident to it. Sometimes called the valence.
    #[inline]
    #[instrument(skip(mesh_graph))]
    pub fn degree(&self, mesh_graph: &MeshGraph) -> usize {
        self.neighbours(mesh_graph).count()
    }

    /// Returns true if this vertex is a boundary vertex, i.e., if it is incident to a boundary edge.
    #[instrument(skip(mesh_graph))]
    pub fn is_boundary(&self, mesh_graph: &MeshGraph) -> bool {
        if let Some(outgoing_he) = self.outgoing_halfedge {
            mesh_graph
                .halfedges
                .get(outgoing_he)
                .or_else(error_none!("Outgoing halfedge not found"))
                .map(|he| he.is_boundary())
                .unwrap_or(false)
        } else {
            error!("Outgoing halfedge is None");
            false
        }
    }

    /// Returns the halfedges that are opposite to this vertex for every incident face to this vertex.
    /// They are ordered counterclockwise.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/one_ring.svg" alt="Connectivity" style="max-width: 50em" />
    #[instrument(skip(mesh_graph))]
    pub fn one_ring(&self, mesh_graph: &MeshGraph) -> impl Iterator<Item = HalfedgeId> {
        self.incoming_halfedges(mesh_graph)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .filter_map(|he| {
                mesh_graph
                    .halfedges
                    .get(he)
                    .or_else(error_none!("Outgoing halfedge not found"))?
                    .cw_rotated_neighbour(mesh_graph)
            })
    }

    /// Returns an iterator over the series of boundary halfedges starting from this vertex.
    ///
    /// If this vertex has no outgoing halfedge or no outgoing boundary halfedges, the iterator is empty.
    pub fn boundary_halfedgdes<'a>(
        &self,
        mesh_graph: &'a MeshGraph,
    ) -> CircularHalfedgesIterator<'a> {
        let Some(out_he_id) = self.outgoing_halfedge else {
            return CircularHalfedgesIterator::empty(mesh_graph);
        };

        let he = unwrap_or_return!(
            mesh_graph.halfedges.get(out_he_id),
            "Halfedge not found",
            CircularHalfedgesIterator::empty(mesh_graph)
        );

        if !he.is_boundary() {
            return CircularHalfedgesIterator::empty(mesh_graph);
        }

        CircularHalfedgesIterator::new(
            self.outgoing_halfedge,
            mesh_graph,
            |he_id, mesh_graph| {
                let end_v_id = mesh_graph
                    .halfedges
                    .get(he_id)
                    .or_else(error_none!("Halfedge not found"))?
                    .end_vertex;
                let next_he_id = mesh_graph
                    .vertices
                    .get(end_v_id)
                    .or_else(error_none!("Vertex not found"))?
                    .outgoing_halfedge?;

                let next_he = mesh_graph
                    .halfedges
                    .get(next_he_id)
                    .or_else(error_none!("Halfedge not found"))?;

                if next_he.is_boundary() {
                    Some(next_he_id)
                } else {
                    None
                }
            },
            usize::MAX,
        )
    }
}
