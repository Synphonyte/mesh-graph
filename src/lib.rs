//! MeshGraph is a halfedge data structure for representing triangle meshes.
//!
//! This is heavily inspired by [SMesh](https://github.com/Bendzae/SMesh) and
//! [OpenMesh](https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh).
//!
//! ## Features
//!
//! - Fast spatial queries using parry3d's Bvh
//! - High performance using slotmap
//! - Easy integration with Bevy game engine using the `bevy` Cargo feature
//! - Good debugging using `rerun` Cargo feature to enable the Rerun integration
//! - Best in class documentation with illustrations
//!
//! ## Usage
//!
//! ```
//! use mesh_graph::{MeshGraph, primitives::IcoSphere};
//!
//! // Create a new mesh
//! let mesh_graph = MeshGraph::from(IcoSphere { radius: 10.0, subdivisions: 2 });
//!
//! // Get some vertex ID and its vertex node
//! let (vertex_id, vertex) = mesh_graph.vertices.iter().next().unwrap();
//!
//! // Iterate over all outgoing halfedges of the vertex
//! for halfedge_id in vertex.outgoing_halfedges(&mesh_graph) {
//!     // do sth
//! }
//!
//! // Get the position of the vertex
//! let position = mesh_graph.positions[vertex_id];
//! ```
//!
//! Check out the crate [freestyle-sculpt](https://github.com/Synphonyte/freestyle-sculpt) for
//! a heavy duty example.
//!
//! ## Connectivity
//!
//! ### Halfedge
//!
//! <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/all.svg" alt="Connectivity" style="max-width: 28em" />
//!
//! ### Vertex
//!
//! <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/all.svg" alt="Connectivity" style="max-width: 50em" />

mod elements;
pub mod integrations;
mod iter;
mod ops;
mod plane_slice;
pub mod primitives;
#[cfg(feature = "rerun")]
mod rerun_impl;
mod selection;
#[cfg(feature = "serde")]
mod serialize;
pub mod utils;

pub use elements::*;
pub use iter::*;
pub use plane_slice::*;
pub use selection::*;

use hashbrown::HashMap;
use parry3d::partitioning::{Bvh, BvhWorkspace};

use glam::Vec3;
use slotmap::{SecondaryMap, SlotMap};
use tracing::{error, instrument};

#[cfg(feature = "rerun")]
lazy_static::lazy_static! {
    pub static ref RR: rerun::RecordingStream = rerun::RecordingStreamBuilder::new("mesh_graph").spawn().unwrap();
}

/// Halfedge data structure for representing triangle meshes.
///
/// Please see the [crate documentation](crate) for more information.
#[derive(Clone, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(from = "crate::serialize::MeshGraphIntermediate")
)]
pub struct MeshGraph {
    /// Acceleration structure for fast spatial queries. Uses parry3d's Bvh to implement some of parry3d's spatial queries.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub bvh: Bvh,
    /// Used in conjunction with the BVH to accelerate spatial queries.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub bvh_workspace: BvhWorkspace,
    /// Used to map indices stored in the BVH to face IDs.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub index_to_face_id: HashMap<u32, FaceId>,
    /// Used to compute the next index for a new face
    #[cfg_attr(feature = "serde", serde(skip))]
    pub next_index: u32,

    /// Maps vertex IDs to their corresponding graph node
    pub vertices: SlotMap<VertexId, Vertex>,
    /// Maps halfedge IDs to their corresponding graph node
    pub halfedges: SlotMap<HalfedgeId, Halfedge>,
    /// Maps face IDs to their corresponding graph node
    pub faces: SlotMap<FaceId, Face>,

    /// Maps vertex IDs to their corresponding positions
    pub positions: SecondaryMap<VertexId, Vec3>,
    /// Maps vertex IDs to their corresponding normals
    pub vertex_normals: Option<SecondaryMap<VertexId, Vec3>>,

    /// Maps vertex IDs to their corresponding outgoing halfedges (not in any particular order)
    pub outgoing_halfedges: SecondaryMap<VertexId, Vec<HalfedgeId>>,
}

impl MeshGraph {
    /// Create a new empty mesh graph
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a triangle mesh graph from vertex positions.
    /// Every three positions represent a triangle.
    ///
    /// Vertices with the same position are merged into a single vertex.
    pub fn triangles(vertex_positions: &[Vec3]) -> Self {
        assert!(
            vertex_positions.len().is_multiple_of(3),
            "Number of vertex positions should be a multiple of 3"
        );

        // Create a map to track unique vertices
        let mut unique_positions: Vec<Vec3> = Vec::with_capacity(vertex_positions.len() / 3);
        let mut face_indices = Vec::with_capacity(vertex_positions.len());

        for vertex_pos in vertex_positions {
            // Check if we've seen this position before using a fuzzy float comparison
            let mut idx = None;
            for (j, pos) in unique_positions.iter().enumerate() {
                const EPSILON: f32 = 1e-5;

                if pos.distance_squared(*vertex_pos) < EPSILON {
                    idx = Some(j);
                    break;
                }
            }

            // Use the existing index or add a new vertex
            let vertex_idx = if let Some(idx) = idx {
                idx
            } else {
                let new_idx = unique_positions.len();
                unique_positions.push(*vertex_pos);

                #[cfg(feature = "rerun")]
                RR.log(
                    "meshgraph/construct/vertices",
                    &rerun::Points3D::new(unique_positions.iter().map(crate::utils::vec3_array)),
                )
                .unwrap();

                new_idx
            };

            // Add to face indices
            face_indices.push(vertex_idx);
        }

        // Use indexed_triangles to create the mesh
        Self::indexed_triangles(&unique_positions, &face_indices)
    }

    /// Create a triangle mesh graph from vertex positions and face indices.
    /// Every chunk of three indices represents a triangle.
    pub fn indexed_triangles(vertex_positions: &[Vec3], face_indices: &[usize]) -> Self {
        let mut mesh_graph = Self {
            bvh: Bvh::new(),
            bvh_workspace: BvhWorkspace::default(),
            index_to_face_id: HashMap::with_capacity(face_indices.len() / 3),
            next_index: 0,

            vertices: SlotMap::with_capacity_and_key(vertex_positions.len()),
            halfedges: SlotMap::with_capacity_and_key(face_indices.len()),
            faces: SlotMap::with_capacity_and_key(face_indices.len() / 3),

            positions: SecondaryMap::with_capacity(vertex_positions.len()),
            vertex_normals: None,
            outgoing_halfedges: SecondaryMap::with_capacity(vertex_positions.len()),
        };

        let mut vertex_ids = Vec::with_capacity(vertex_positions.len());

        for pos in vertex_positions {
            vertex_ids.push(mesh_graph.insert_vertex(*pos));
        }

        for chunk in face_indices.chunks_exact(3) {
            let a = vertex_ids[chunk[0]];
            let b = vertex_ids[chunk[1]];
            let c = vertex_ids[chunk[2]];

            if a == b || b == c || c == a {
                #[cfg(feature = "rerun")]
                RR.log(
                    "meshgraph/construct/zero_face",
                    &rerun::Points3D::new(
                        [
                            mesh_graph.positions[a],
                            mesh_graph.positions[b],
                            mesh_graph.positions[c],
                        ]
                        .iter()
                        .map(crate::utils::vec3_array),
                    ),
                )
                .unwrap();

                continue;
            }

            let (he_a_id, _) = mesh_graph.insert_or_get_edge(a, b);
            let (he_b_id, _) = mesh_graph.insert_or_get_edge(b, c);
            let (he_c_id, _) = mesh_graph.insert_or_get_edge(c, a);

            let _face_id = mesh_graph.insert_face(he_a_id, he_b_id, he_c_id);

            // #[cfg(feature = "rerun")]
            // {
            //     mesh_graph.log_hes_rerun(
            //         "indexed_triangles/halfedges",
            //         &mesh_graph.halfedges.keys().collect::<Vec<_>>(),
            //     );
            // }
        }

        mesh_graph.make_all_outgoing_halfedges_boundary_if_possible();
        mesh_graph.rebuild_bvh();

        mesh_graph
    }

    /// Computes the vertex normals by averaging over the computed face normals
    #[instrument(skip(self))]
    pub fn compute_vertex_normals(&mut self) {
        let mut normals = SecondaryMap::with_capacity(self.vertices.len());

        for face in self.faces.values() {
            let ha_a_id = face.halfedge;
            let he_a = self.halfedges[ha_a_id];

            let he_b_id = he_a
                .next
                .expect("Halfedge has definitely a face and thus a next halfedge");
            let he_b = self.halfedges[he_b_id];

            let a = match he_a.start_vertex(self) {
                Some(v) => v,
                None => {
                    error!("Start vertex not found");
                    continue;
                }
            };
            let b = he_a.end_vertex;
            let c = he_b.end_vertex;

            let diff_a = self.positions[c] - self.positions[a];
            let diff_b = self.positions[c] - self.positions[b];

            // TODO : normalizing necessary here?
            let face_normal = diff_a.cross(diff_b);

            *normals.entry(a).unwrap().or_default() += face_normal;
            *normals.entry(b).unwrap().or_default() += face_normal;
            *normals.entry(c).unwrap().or_default() += face_normal;
        }

        self.vertex_normals = Some(normals);
        self.normalize_vertex_normals();
    }

    /// Ensures that the vertex normals are all normalized
    pub fn normalize_vertex_normals(&mut self) {
        if let Some(normals) = &mut self.vertex_normals {
            for normal in normals.values_mut() {
                *normal = normal.normalize();
            }
        }
    }

    /// Calls the `optimize_incremental` method of the BVH.
    #[inline]
    pub fn optimize_bvh_incremental(&mut self) {
        self.bvh.optimize_incremental(&mut self.bvh_workspace);
    }

    /// Recomputes the bounding boxes of the BVH. This is necessary when the mesh is modified.
    #[inline]
    pub fn refit_bvh(&mut self) {
        self.bvh.refit(&mut self.bvh_workspace);
    }

    /// Rebuilds the BVH from scratch
    #[inline]
    pub fn rebuild_bvh(&mut self) {
        for face in self.faces.values() {
            self.bvh
                .insert_or_update_partially(face.aabb(self), face.index, 0.0);
        }
        self.bvh
            .rebuild(&mut self.bvh_workspace, Default::default());
    }
}
