use crate::MeshGraph;

use super::{FaceId, HalfedgeId, VertexId};

/// A vertex is an corner point of a face.
///
/// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/all.svg" alt="Connectivity" style="max-width: 40em" />
#[derive(Debug, Default, Clone, Copy)]
pub struct Vertex {
    /// One of the halfedges with this vertex as start point.
    /// If possible this is a boundary halfedge, i.e. it has no associated face.
    ///
    /// After the mesh graph is constructed correctly, this is always `Some`.
    pub outgoing_halfedge: Option<HalfedgeId>,
}

impl Vertex {
    /// One of the incoming halfedges of this vertex.
    pub fn incoming_halfedge(&self, mesh_graph: &MeshGraph) -> HalfedgeId {
        mesh_graph.halfedges[self.outgoing_halfedge()].twin()
    }

    #[inline]
    /// One of the outgoing halfedges of this vertex.
    pub fn outgoing_halfedge(&self) -> HalfedgeId {
        self.outgoing_halfedge
            .expect("Outgoing halfedge should be connected")
    }

    // TODO : create iterator instead of returning a Vec
    /// Returns all halfedges that point away from this vertex.
    pub fn outgoing_halfedges(&self, mesh_graph: &MeshGraph) -> Vec<HalfedgeId> {
        let mut edges = Vec::new();

        let mut current_edge = self.outgoing_halfedge();

        edges.push(current_edge);

        while let Some(cw) = mesh_graph.halfedges[current_edge].cw_rotated_neighbour(mesh_graph) {
            if cw == edges[0] {
                break;
            }

            edges.push(cw);
            current_edge = cw;
        }

        edges
    }

    /// Returns all halfedges that point towards this vertex
    pub fn incoming_halfedges(&self, mesh_graph: &MeshGraph) -> Vec<HalfedgeId> {
        self.outgoing_halfedges(mesh_graph)
            .into_iter()
            .map(|he_id| mesh_graph.halfedges[he_id].twin())
            .collect()
    }

    // TODO : create iterator instead of returning a Vec
    /// Returns all faces incident to this vertex.
    pub fn faces(&self, mesh_graph: &MeshGraph) -> Vec<FaceId> {
        self.outgoing_halfedges(mesh_graph)
            .into_iter()
            .filter_map(|he| mesh_graph.halfedges[he].face)
            .collect()
    }

    // TODO : create iterator instead of returning a Vec
    /// Returns all neighbouring (connected through an edge) vertices of this vertex.
    pub fn neighbours(&self, mesh_graph: &MeshGraph) -> Vec<VertexId> {
        self.outgoing_halfedges(mesh_graph)
            .into_iter()
            .map(|he| mesh_graph.halfedges[he].end_vertex)
            .collect()
    }

    /// The degree of this vertex, i.e., the number of edges incident to it. Sometimes called the valence.
    #[inline]
    pub fn degree(&self, mesh_graph: &MeshGraph) -> usize {
        self.neighbours(mesh_graph).len()
    }

    #[inline]
    pub fn is_boundary(&self, mesh_graph: &MeshGraph) -> bool {
        mesh_graph.halfedges[self.outgoing_halfedge()].is_boundary()
    }

    /// Returns the halfedges that are opposite to this vertex for every incident face to this vertex.
    /// They are ordered counterclockwise.
    pub fn one_ring(&self, mesh_graph: &MeshGraph) -> Vec<HalfedgeId> {
        self.incoming_halfedges(mesh_graph)
            .into_iter()
            .rev()
            .filter_map(|he| mesh_graph.halfedges[he].cw_rotated_neighbour(mesh_graph))
            .collect()
    }
}
