use glam::Vec3;
use parry3d::{bounding_volume::Aabb, na::Point3};
use tracing::{error, instrument};

use crate::{CircularHalfedgesIterator, MeshGraph, error_none};

use super::{FaceId, HalfedgeId, VertexId};

#[derive(Default, Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Face {
    /// One of the halfedges of the face.
    /// Serves as a starting point for traversing the face's edges and vertices
    pub halfedge: HalfedgeId,

    /// The index of the face in the BVH
    pub index: u32,

    /// The associated face id
    pub id: FaceId,
}

impl Face {
    /// Returns the three halfedges that form this face
    #[instrument(skip(mesh_graph))]
    pub fn halfedges<'a>(&self, mesh_graph: &'a MeshGraph) -> CircularHalfedgesIterator<'a> {
        CircularHalfedgesIterator::new(Some(self.halfedge), mesh_graph, |he, mesh_graph| {
            mesh_graph
                .halfedges
                .get(he)
                .or_else(error_none!("Halfedge not found"))?
                .next
        })
    }

    /// Returns the three corner vertices of this face.
    #[instrument(skip(mesh_graph))]
    pub fn vertices(&self, mesh_graph: &MeshGraph) -> impl Iterator<Item = VertexId> {
        self.halfedges(mesh_graph).filter_map(|he| {
            mesh_graph
                .halfedges
                .get(he)
                .or_else(error_none!("Halfedge not found"))
                .map(|he| he.end_vertex)
        })
    }

    /// Center positions of this face.
    #[instrument(skip(mesh_graph))]
    pub fn center(&self, mesh_graph: &MeshGraph) -> Vec3 {
        let mut sum = Vec3::ZERO;

        for vertex in self.vertices(mesh_graph) {
            if let Some(position) = mesh_graph.positions.get(vertex) {
                sum += position;
            } else {
                error!("Vertex position not found");
            }
        }

        sum / 3.0
    }

    /// Compute the parry Aabb of this triangle
    #[instrument(skip(mesh_graph))]
    pub fn aabb(&self, mesh_graph: &MeshGraph) -> Aabb {
        Aabb::from_points(self.vertices(mesh_graph).filter_map(|v| {
            mesh_graph
                .positions
                .get(v)
                .or_else(error_none!("Position not found"))
                .map(|p| Point3::new(p.x, p.y, p.z))
        }))
    }
}
