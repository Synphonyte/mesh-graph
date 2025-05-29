use glam::Vec3;

use crate::MeshGraph;

use super::{FaceId, HalfedgeId, VertexId};

#[derive(Default, Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Face {
    /// One of the halfedges of the face.
    /// Serves as a starting point for traversing the face's edges and vertices
    pub halfedge: HalfedgeId,

    /// The index of the face in the qbvh
    pub index: usize,

    /// The associated face id
    pub id: FaceId,
}

impl parry3d::partitioning::IndexedData for Face {
    fn default() -> Self {
        Default::default()
    }

    fn index(&self) -> usize {
        self.index
    }
}

impl Face {
    // TODO : create iterator instead of returning a Vec
    /// Returns the three halfedges that form this face
    pub fn halfedges(&self, mesh_graph: &MeshGraph) -> Vec<HalfedgeId> {
        // TODO :this currently only works for triangle meshes
        let mut edges = Vec::with_capacity(3);
        edges.push(self.halfedge);

        let he = mesh_graph.halfedges[self.halfedge];
        let next_id = he.next.unwrap();
        edges.push(next_id);
        edges.push(mesh_graph.halfedges[next_id].next.unwrap());

        edges
    }

    // TODO : create iterator instead of returning a Vec
    /// Returns the three corner vertices of this face.
    pub fn vertices(&self, mesh_graph: &MeshGraph) -> Vec<VertexId> {
        let mut vertices = Vec::with_capacity(3);

        for halfedge in self.halfedges(mesh_graph) {
            vertices.push(mesh_graph.halfedges[halfedge].end_vertex);
        }

        vertices
    }

    /// Center positions of this face.
    pub fn center(&self, mesh_graph: &MeshGraph) -> Vec3 {
        let mut sum = Vec3::ZERO;
        for vertex in self.vertices(mesh_graph) {
            sum += mesh_graph.positions[vertex];
        }
        sum / 3.0
    }
}
