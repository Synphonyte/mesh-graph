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

mod access;
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
pub use ops::*;
pub use plane_slice::*;
pub use selection::*;

use hashbrown::HashMap;
use parry3d::partitioning::{Bvh, BvhWorkspace};

use glam::Vec3;
use slotmap::{SecondaryMap, SlotMap};

use crate::elements::FaceId;
use tracing::{error, instrument};

use crate::utils::unwrap_or_return;

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
    #[cfg_attr(feature = "serde", serde(skip))]
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
    pub fn triangles(vertex_positions: &[Vec3]) -> Option<Self> {
        if !vertex_positions.len().is_multiple_of(3) {
            return None;
        }

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
        Some(Self::indexed_triangles(&unique_positions, &face_indices))
    }

    /// Create a triangle mesh graph from vertex positions, face indices,
    /// and a custom vertex attribute.
    #[instrument]
    pub fn indexed_triangles_with_custom_attribute<T>(
        vertex_positions: &[Vec3],
        face_indices: &[usize],
        custom_attribute: &[T],
    ) -> (Self, SecondaryMap<VertexId, T>)
    where
        T: Clone + std::fmt::Debug,
    {
        let (mesh_graph, vertex_ids) =
            Self::indexed_triangles_and_vertex_ids(vertex_positions, face_indices);

        let mut custom_attribute_map = SecondaryMap::with_capacity(custom_attribute.len());
        for (attr, vertex_id) in custom_attribute.iter().zip(vertex_ids) {
            custom_attribute_map.insert(vertex_id, attr.clone());
        }

        (mesh_graph, custom_attribute_map)
    }

    /// Create a triangle mesh graph from vertex positions and face indices.
    /// Every chunk of three indices represents a triangle.
    #[inline]
    pub fn indexed_triangles(vertex_positions: &[Vec3], face_indices: &[usize]) -> Self {
        Self::indexed_triangles_and_vertex_ids(vertex_positions, face_indices).0
    }

    /// Create a triangle mesh graph from vertex positions and face indices,
    /// returning the graph and a list of vertex IDs in the same order as `vertex_positions`.
    #[instrument]
    pub fn indexed_triangles_and_vertex_ids(
        vertex_positions: &[Vec3],
        face_indices: &[usize],
    ) -> (Self, Vec<VertexId>) {
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
            vertex_ids.push(mesh_graph.add_vertex(*pos));
        }

        for chunk in face_indices.as_chunks::<3>().0 {
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

            // Vertices have already been added to the mesh graph, so we can safely use `unwrap()` here
            let he_a_id = mesh_graph.add_or_get_edge(a, b).unwrap().start_to_end_he_id;
            let he_b_id = mesh_graph.add_or_get_edge(b, c).unwrap().start_to_end_he_id;
            let he_c_id = mesh_graph.add_or_get_edge(c, a).unwrap().start_to_end_he_id;

            let _face_id = mesh_graph.add_face(he_a_id, he_b_id, he_c_id);
        }

        mesh_graph.make_all_outgoing_halfedges_boundary_if_possible();
        mesh_graph.rebuild_bvh();

        (mesh_graph, vertex_ids)
    }

    /// Nulls the `twin` field of every surviving halfedge pointing at one of the
    /// `removed_ids`. Removing a halfedge must never leave a dangling twin reference
    /// behind.
    ///
    /// Local O(|removed_ids|) implementation (replaces the previous O(H) full scan):
    /// twin pointers are only ever written as symmetric pairs (`make_twins`,
    /// `add_or_get_edge`, the weld/flap re-pairs, ...) and every re-pair severs the
    /// old partners' back-references, so for every removed halfedge its own twin
    /// partner is the only surviving halfedge whose `twin` field can reference it.
    /// The partner's back-pointer is nulled here; a back-pointer is only cleared
    /// when it provably points at the removed id, which keeps the effect identical
    /// to the full scan. Removed halfedges must still be present at the call site
    /// (all callers clear before removing) except for `remove_only_halfedge_and_twin`
    /// whose partner is already gone.
    ///
    /// Debug probe (keep for the layer-4 hunt): with `MESH_GRAPH_DANGLING_CHECK=1`
    /// reports once when a removal takes a halfedge that still belongs to a live face
    /// whose other members survive, i.e. a removal that breaks a face chain.
    pub(crate) fn clear_twins_to(&mut self, removed_ids: &[HalfedgeId]) {
        if removed_ids.is_empty() {
            return;
        }

        static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        static REPORTED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if *ENABLED.get_or_init(|| std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_some())
            && !*REPORTED.get_or_init(|| false)
        {
            // Mark as reported *before* scanning so this expensive probe runs at most once.
            let _ = REPORTED.set(true);
            for id in removed_ids {
                if let Some(he) = self.halfedges.get(*id)
                    && let Some(face_id) = he.face
                    && self.faces.contains_key(face_id)
                {
                    let other_members: Vec<HalfedgeId> = self
                        .halfedges
                        .iter()
                        .filter(|(h_id, h)| {
                            h.face == Some(face_id)
                                && *h_id != *id
                                && !removed_ids.contains(h_id)
                        })
                        .map(|(h_id, _)| h_id)
                        .take(4)
                        .collect();
                    if !other_members.is_empty() {
                        eprintln!(
                            "REMOVING LIVE-FACE MEMBER {id:?} of face {face_id:?} (surviving members {other_members:?})"
                        );
                        eprintln!("{}", std::backtrace::Backtrace::force_capture());
                    }
                }
            }
        }

        for &removed_id in removed_ids {
            let Some(he) = self.halfedges.get(removed_id) else {
                continue; // already removed (`remove_only_halfedge_and_twin` clears after its partner)
            };
            let Some(twin_id) = he.twin else {
                continue;
            };
            if removed_ids.contains(&twin_id) {
                continue; // the partner is being removed in the same batch, nothing survives to clear
            }
            let Some(twin) = self.halfedges.get_mut(twin_id) else {
                continue;
            };
            if twin.twin == Some(removed_id) {
                twin.twin = None;
            }
        }
    }

    /// Computes the vertex normal from neighboring faces
    pub fn compute_vertex_normal(&mut self, vertex_id: VertexId) {
        if self.vertex_normals.is_none() {
            return;
        }

        let vertex = unwrap_or_return!(self.vertices.get(vertex_id), "Vertex not found");

        let mut normal = Vec3::ZERO;

        for face_id in vertex.faces(self) {
            let face = unwrap_or_return!(self.faces.get(face_id), "Face not found");
            let face_normal = face.normal(self);
            normal += unwrap_or_return!(face_normal, "Face normal not found");
        }

        self.vertex_normals
            .as_mut()
            .unwrap()
            .insert(vertex_id, normal.try_normalize().unwrap_or(Vec3::ZERO));
    }

    /// Computes the vertex normals by averaging over the computed face normals
    #[instrument(skip(self))]
    pub fn compute_vertex_normals(&mut self) {
        let mut normals = SecondaryMap::with_capacity(self.vertices.len());

        for face in self.faces.values() {
            let Some(&he_a) = self.halfedges.get(face.halfedge) else {
                error!("Halfedge not found");
                continue;
            };

            let Some(he_b_id) = he_a.next else {
                error!("Halfedge has no next halfedge");
                continue;
            };
            let Some(he_b) = self.halfedges.get(he_b_id) else {
                error!("Next halfedge not found");
                continue;
            };

            let a = match he_a.start_vertex(self) {
                Some(v) => v,
                None => {
                    error!("Start vertex not found");
                    continue;
                }
            };
            let b = he_a.end_vertex;
            let c = he_b.end_vertex;

            let (Some(pos_a), Some(pos_b), Some(pos_c)) = (
                self.positions.get(a),
                self.positions.get(b),
                self.positions.get(c),
            ) else {
                continue;
            };

            let diff_a = pos_c - pos_a;
            let diff_b = pos_c - pos_b;

            // TODO : normalizing necessary here?
            let face_normal = diff_a.cross(diff_b);

            for v_id in [a, b, c] {
                let Some(entry) = normals.entry(v_id) else {
                    continue;
                };
                *entry.or_default() += face_normal;
            }
        }

        self.vertex_normals = Some(normals);
        self.normalize_vertex_normals();
    }

    /// Ensures that the vertex normals are all normalized
    pub fn normalize_vertex_normals(&mut self) {
        if let Some(normals) = &mut self.vertex_normals {
            for normal in normals.values_mut() {
                *normal = normal.normalize_or_zero();
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
        self.bvh = Bvh::new();
        self.bvh_workspace = BvhWorkspace::default();

        for face in self.faces.values() {
            self.bvh
                .insert_or_update_partially(face.aabb(self), face.index, 0.0);
        }
        self.bvh
            .rebuild(&mut self.bvh_workspace, Default::default());
    }

    #[instrument(skip_all)]
    /// Repairs the redundant `Halfedge::face` cache from the halfedge chains, which are
    /// the ground truth for face membership (just like `rebuild_outgoing_halfedges` is for
    /// the per-vertex lists). Stale `.face` pointers (e.g. after flap-removal twin
    /// re-pairs) make later operations subdivide the wrong faces and degenerate the mesh.
    /// Also clears `.face` on halfedges that are no longer reachable from any face's chain.
    ///
    /// O(F + H), meant to be called once per operation that rewires faces.
    pub fn repair_face_pointers(&mut self) {
        let face_ids: Vec<FaceId> = self.faces.keys().collect();
        let mut visited: hashbrown::HashMap<HalfedgeId, FaceId> = hashbrown::HashMap::new();

        for face_id in face_ids {
            let Some(start_he) = self.faces.get(face_id).map(|f| f.halfedge) else {
                continue;
            };

            let mut he_id = start_he;
            for _ in 0..32 {
                let Some(he) = self.halfedges.get_mut(he_id) else {
                    break;
                };
                he.face = Some(face_id);
                visited.insert(he_id, face_id);

                let Some(next) = he.next else {
                    break;
                };
                if next == start_he {
                    break;
                }
                he_id = next;
            }
        }

        // Halfedges that claim a face but are not reachable from that face's chain are
        // orphans left behind by re-links (e.g. flap twin re-pairs). Clear their stale
        // `.face` so they read as boundary, which all traversals handle.
        let orphan_ids: Vec<HalfedgeId> = self
            .halfedges
            .iter()
            .filter(|(he_id, he)| he.face.is_some() && !visited.contains_key(he_id))
            .map(|(he_id, _)| he_id)
            .collect();

        for he_id in orphan_ids {
            if let Some(he) = self.halfedges.get_mut(he_id) {
                he.face = None;
            }
        }
    }

    pub fn rebuild_outgoing_halfedges(&mut self) {
        self.outgoing_halfedges.clear();

        // Keep an (empty) list entry for every live vertex: a live vertex without any
        // halfedges (e.g. a leftover isolated vertex after a cleanup) must still have a list
        // entry so lookups like `halfedge_from_to` answer "no edge" instead of failing with
        // "Start vertex not found".
        for vertex_id in self.vertices.keys() {
            self.outgoing_halfedges.insert(vertex_id, Vec::new());
        }

        for halfedge in self.halfedges.values() {
            let Some(twin_id) = halfedge.twin else {
                error!("Halfedge has no twin");
                continue;
            };

            let Some(entry) = self.outgoing_halfedges.entry(halfedge.end_vertex) else {
                error!("Vertex key invalid");
                continue;
            };

            entry.or_default().push(twin_id);
        }

        // Normalize the per-vertex seed pointers (`vertices[v].outgoing_halfedge`). Stale
        // seeds pointing at removed halfedges make ring traversals (`one_ring`, faces, ...)
        // yield dead ids, which panics callers that index them. Only replace the seed when
        // it is dead, to keep ring iteration order stable in the common case.
        for (v_id, vertex) in &mut self.vertices {
            let stored_seed = vertex.outgoing_halfedge;
            let live_seed = stored_seed.filter(|he| self.halfedges.contains_key(*he));
            vertex.outgoing_halfedge = live_seed.or_else(|| {
                self.outgoing_halfedges.get(v_id).and_then(|list| list.first().copied())
            });
        }
    }
}
