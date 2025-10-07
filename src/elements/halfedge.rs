use tracing::{error, instrument};

use crate::{MeshGraph, error_none};

use super::{FaceId, HalfedgeId, VertexId};

/// A directional edge that points from one vertex to another and is (optionally) part of a face.
/// If it's not part of a face, it's called a boundary halfedge.
///
/// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/all.svg" alt="Connectivity" style="max-width: 28em" />
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Halfedge {
    /// The vertex that this halfedge points to.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/end_vertex.svg" alt="Connectivity" style="max-width: 28em" />
    pub end_vertex: VertexId,

    /// The face associated to this halfedge. `None` if `self` is a boundary halfedge.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/face.svg" alt="Connectivity" style="max-width: 28em" />
    pub face: Option<FaceId>,

    /// This is the halfedge opposite.
    /// It points backwards compared to this halfedge (from end_vertex to start_vertex).
    /// After the mesh graph is constructed, this field is always `Some(...)`, meaning
    /// that every halfedge has a twin.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/twin.svg" alt="Connectivity" style="max-width: 28em" />
    pub twin: Option<HalfedgeId>,

    /// The next halfedge in the face. `None` if `self` is a boundary halfedge.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/next.svg" alt="Connectivity" style="max-width: 28em" />
    pub next: Option<HalfedgeId>,
}

impl Halfedge {
    /// Start vertex from which this halfedge points away
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/start_vertex.svg" alt="Connectivity" style="max-width: 28em" />
    #[instrument(skip(mesh_graph))]
    pub fn start_vertex(&self, mesh_graph: &MeshGraph) -> Option<VertexId> {
        mesh_graph
            .halfedges
            .get(self.twin.or_else(error_none!("No twin"))?)
            .or_else(error_none!("Twin halfedge not found"))
            .map(|t| t.end_vertex)
    }

    /// Previous halfedge that shares the same face. `None` if `self` is a boundary halfedge.
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/prev.svg" alt="Connectivity" style="max-width: 28em" />
    #[instrument(skip(mesh_graph))]
    pub fn prev(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        // TODO : this only works for triangle meshes
        self.next.and_then(|next_id| {
            mesh_graph
                .halfedges
                .get(next_id)
                .or_else(error_none!("Next halfedge not found"))
                .and_then(|h| h.next)
        })
    }

    /// In counter-clockwise order next halfedge that has the same start vertex
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/ccw_rotated_neighbour.svg" alt="Connectivity" style="max-width: 28em" />
    #[instrument(skip(mesh_graph))]
    pub fn ccw_rotated_neighbour(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        self.prev(mesh_graph).and_then(|prev| {
            mesh_graph
                .halfedges
                .get(prev)
                .or_else(error_none!("Previous halfedge not found"))
                .and_then(|h| h.twin.or_else(error_none!("Twin missing")))
        })
    }

    /// In clockwise order next halfedge that has the same start vertex
    ///
    /// <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/cw_rotated_neighbour.svg" alt="Connectivity" style="max-width: 28em" />
    #[instrument(skip(mesh_graph))]
    pub fn cw_rotated_neighbour(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        mesh_graph
            .halfedges
            .get(self.twin.or_else(error_none!("Twin missing"))?)
            .and_then(|h| h.next)
    }

    /// Length of the halfedge squared.
    #[instrument(skip(mesh_graph))]
    pub fn length_squared(&self, mesh_graph: &MeshGraph) -> f32 {
        self.len_sqr_inner(mesh_graph).unwrap_or_else(|| {
            error!("Halfedge invalid. Defaulting to zero length");
            0.0
        })
    }

    fn len_sqr_inner(&self, mesh_graph: &MeshGraph) -> Option<f32> {
        let start = mesh_graph.positions.get(self.start_vertex(mesh_graph)?)?;
        let end = mesh_graph.positions.get(self.end_vertex)?;

        Some(start.distance_squared(*end))
    }

    /// Length of the halfedge.
    #[inline(always)]
    pub fn length(&self, mesh_graph: &MeshGraph) -> f32 {
        self.length_squared(mesh_graph).sqrt()
    }

    /// Returns `true` if there is no face adjacent to this halfedge.
    #[inline]
    pub fn is_boundary(&self) -> bool {
        self.face.is_none()
    }

    /// Returns `true` if the start or end vertex is a boundary vertex.
    #[instrument(skip(mesh_graph))]
    pub fn is_adjacent_to_boundary(&self, mesh_graph: &MeshGraph) -> bool {
        if let Some(start_vertex) = self.start_vertex(mesh_graph) {
            mesh_graph
                .vertices
                .get(start_vertex)
                .or_else(error_none!("Start vertex not found"))
                .map(|v| v.is_boundary(mesh_graph))
                .unwrap_or(false)
                || mesh_graph
                    .vertices
                    .get(self.end_vertex)
                    .or_else(error_none!("End vertex not found"))
                    .map(|v| v.is_boundary(mesh_graph))
                    .unwrap_or(false)
        } else {
            error!("Start vertex ID not found");
            false
        }
    }
}
