use crate::MeshGraph;

use super::{FaceId, HalfedgeId, VertexId};

/// A directional edge that points from one vertex to another and is (optionally) part of a face.
/// If it's not part of a face, it's called a boundary halfedge.
///
/// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/all.svg)
#[derive(Debug, Clone, Copy)]
pub struct Halfedge {
    /// The vertex that this halfedge points to.
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/end_vertex.svg)
    pub end_vertex: VertexId,

    /// The face associated to this halfedge. `None` if this is a boundary halfedge.
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/face.svg)
    pub face: Option<FaceId>,

    /// This is the halfedge opposite.
    /// It points backwards compared to this halfedge (from end_vertex to start_vertex).
    /// After the mesh graph is constructed, this field is always `Some(...)`, meaning
    /// that every halfedge has a twin.
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/twin.svg)
    pub twin: Option<HalfedgeId>,

    /// The next halfedge in the face. `None` if this is a boundary halfedge.
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/next.svg)
    pub next: Option<HalfedgeId>,
}

impl Halfedge {
    /// Start vertex from which this halfedge points away
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/start_vertex.svg)
    pub fn start_vertex(&self, mesh_graph: &MeshGraph) -> VertexId {
        mesh_graph.halfedges[self.twin()].end_vertex
    }

    /// Same as the field `twin` but expects there to be a `Some` which is the case if
    /// the mesh graph is constructed correctly.
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/twin.svg)
    #[inline]
    pub fn twin(&self) -> HalfedgeId {
        self.twin.expect("Twin should be connected by now")
    }

    /// Previous halfedge that shares the same face
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/prev.svg)
    pub fn prev(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        // TODO : this only works for triangle meshes
        self.next
            .map(|next_id| mesh_graph.halfedges[next_id].next.unwrap())
    }

    /// In counter-clockwise order next halfedge that has the same start vertex
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/ccw_rotated_neighbour.svg)
    pub fn ccw_rotated_neighbour(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        self.prev(mesh_graph)
            .map(|prev| mesh_graph.halfedges[prev].twin())
    }

    /// In clockwise order next halfedge that has the same start vertex
    ///
    /// ![Connectivity](https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/cw_rotated_neighbour.svg)
    pub fn cw_rotated_neighbour(&self, mesh_graph: &MeshGraph) -> Option<HalfedgeId> {
        mesh_graph.halfedges[self.twin()].next
    }

    /// Length of the halfedge squared.
    pub fn length_squared(&self, mesh_graph: &MeshGraph) -> f32 {
        let start = mesh_graph.positions[self.start_vertex(mesh_graph)];
        let end = mesh_graph.positions[self.end_vertex];

        start.distance_squared(end)
    }

    /// Returns `true` if there is no face adjacent to this halfedge.
    #[inline]
    pub fn is_boundary(&self) -> bool {
        self.face.is_none()
    }

    /// Returns `true` if the start or end vertex is a boundary vertex.
    pub fn is_adjacent_to_boundary(&self, mesh_graph: &MeshGraph) -> bool {
        mesh_graph.vertices[self.start_vertex(mesh_graph)].is_boundary(mesh_graph)
            || mesh_graph.vertices[self.end_vertex].is_boundary(mesh_graph)
    }
}
