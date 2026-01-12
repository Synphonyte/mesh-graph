use glam::Vec3;
use itertools::Itertools;
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

    /// Returns an iterator over the vertex positions of this face.
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
        let positions = self.vertex_positions(mesh_graph).collect_vec();

        if positions.len() < 3 {
            error!("Face has less than 3 vertex positions");
            return None;
        }

        let a = positions[1] - positions[0];
        let b = positions[2] - positions[0];
        Some(a.cross(b).normalize())
    }

    /// Wether this triangle is degenerate.
    #[instrument(skip(mesh_graph))]
    pub fn is_degenerate(&self, mesh_graph: &MeshGraph, epsilon_sqr: f32) -> bool {
        let positions = self.vertex_positions(mesh_graph).collect_vec();

        if positions.len() < 3 {
            error!("Face has less than 3 vertex positions");
            return true;
        }

        let p0 = positions[0];
        let p1 = positions[1];
        let p2 = positions[2];

        // Check for coincident vertices
        if p0.distance_squared(p1) < epsilon_sqr
            || p0.distance_squared(p2) < epsilon_sqr
            || p1.distance_squared(p2) < epsilon_sqr
        {
            return true;
        }

        // Check for collinear vertices using cross product
        let a = p1 - p0;
        let b = p2 - p0;
        let cross = a.cross(b);

        // Triangle is degenerate if cross product magnitude is very small
        cross.length_squared() < epsilon_sqr
    }

    #[instrument(skip(mesh_graph))]
    pub fn halfedge_between(
        &self,
        vertex_id1: VertexId,
        vertex_id2: VertexId,
        mesh_graph: &MeshGraph,
    ) -> Option<HalfedgeId> {
        for he_id in self.halfedges(mesh_graph) {
            let he = mesh_graph
                .halfedges
                .get(he_id)
                .or_else(error_none!("Halfedge not found"))?;

            if he.end_vertex == vertex_id1 || he.end_vertex == vertex_id2 {
                let he_start_vertex_id = he
                    .start_vertex(mesh_graph)
                    .or_else(error_none!("Start vertex not found"))?;
                if he_start_vertex_id == vertex_id1 || he_start_vertex_id == vertex_id2 {
                    return Some(he_id);
                }
            }
        }

        None
    }
}
