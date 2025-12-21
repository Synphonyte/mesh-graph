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
        self.vertex_positions(mesh_graph).sum::<Vec3>() / 3.0
    }

    /// Compute the parry Aabb of this triangle
    #[instrument(skip(mesh_graph))]
    pub fn aabb(&self, mesh_graph: &MeshGraph) -> Aabb {
        Aabb::from_points(
            self.vertex_positions(mesh_graph)
                .map(|p| Point3::new(p.x, p.y, p.z)),
        )
    }

    #[instrument(skip(mesh_graph))]
    pub fn vertex_positions(&self, mesh_graph: &MeshGraph) -> impl Iterator<Item = Vec3> {
        self.vertices(mesh_graph).filter_map(|v| {
            mesh_graph
                .positions
                .get(v)
                .or_else(error_none!("Position not found"))
                .copied()
        })
    }

    /// Compute the normal of this triangle
    #[instrument(skip(mesh_graph))]
    pub fn normal(&self, mesh_graph: &MeshGraph) -> Option<Vec3> {
        let positions = self.vertex_positions(mesh_graph).collect::<Vec<_>>();

        if positions.len() < 3 {
            error!("Face has less than 3 vertex positions");
            return None;
        }

        let a = positions[1] - positions[0];
        let b = positions[2] - positions[0];
        Some(a.cross(b).normalize())
    }
}
