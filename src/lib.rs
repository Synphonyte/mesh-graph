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

use tracing::{error, instrument};

use crate::utils::unwrap_or_return;

#[cfg(feature = "instrumentation")]
// Debug support for the layer-4 corruption hunt (`MESH_GRAPH_DANGLING_CHECK=1`):
// the name of the operation currently running on this thread. Set by the public
// ops (collapse/subdivide/merge/...) and recorded next to every face removal in
// [`record_face_death`], so a corruption report can name the op that killed a face.
thread_local! {
    static CURRENT_OP: std::cell::Cell<&'static str> = const { std::cell::Cell::new("unknown") };
}

/// Records the current op name on this thread (see [`CURRENT_OP`]).
#[cfg(feature = "instrumentation")]
#[inline]
pub(crate) fn set_current_op(op: &'static str) {
    if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_some() {
        CURRENT_OP.with(|cell| cell.set(op));
    }
}

/// A small ring of the most recent face removals: `(op that removed it, face id)`.
/// Only populated when `MESH_GRAPH_DANGLING_CHECK=1` (see [`record_face_death`]).
#[cfg(feature = "instrumentation")]
static FACE_DEATH_LEDGER: std::sync::Mutex<std::collections::VecDeque<(FaceId, &'static str)>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

#[cfg(feature = "instrumentation")]
const FACE_DEATH_LEDGER_CAP: usize = 64;

/// Records a face removal for the corruption hunt. Cheap when the env flag is unset.
#[cfg(feature = "instrumentation")]
#[inline]
pub(crate) fn record_face_death(face_id: FaceId) {
    if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_none() {
        return;
    }
    let op = CURRENT_OP.with(|cell| cell.get());
    if let Ok(mut ledger) = FACE_DEATH_LEDGER.lock() {
        ledger.push_back((face_id, op));
        if ledger.len() > FACE_DEATH_LEDGER_CAP {
            ledger.pop_front();
        }
    }
}

// Per-op boundary capture for the hole-delta probe (`MESH_GRAPH_HOLE_CHECK=1`
// on top of `MESH_GRAPH_DANGLING_CHECK`): the undirected `(he, twin)` pairs of
// all live halfedges with `face = None`, snapshotted at the entry of each probed
// op (collapse/subdivide/merge/...). Every probed op is required to leave this
// exact set untouched — a topology op that opens or closes the surface mid-op is
// corruption (see `probe_chain_integrity`). Thread-local because the unit tests
// run ops of independent meshes on parallel threads.
#[cfg(feature = "instrumentation")]
thread_local! {
    static OP_BOUNDARY: std::cell::RefCell<Option<hashbrown::HashSet<(HalfedgeId, HalfedgeId)>>> =
        const { std::cell::RefCell::new(None) };
}

/// Must be called at the entry of every probed op (next to [`set_current_op`]).
/// Snapshots the boundary edge set so the op-end probe can compare against it.
/// Unprobed ops in between (the `remove_face`-family punch cuts) never update the
/// snapshot, so a hole cut between two probed ops is never blamed on either of
/// them — each probed op is only held accountable for its own delta.
#[cfg(feature = "instrumentation")]
#[inline]
pub(crate) fn probe_chain_begin(mesh: &MeshGraph) {
    if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_none()
        || std::env::var_os("MESH_GRAPH_HOLE_CHECK").is_none()
    {
        return;
    }
    let boundary = mesh.boundary_edge_set();
    OP_BOUNDARY.with(|b| *b.borrow_mut() = Some(boundary));
}

/// Prints the added/removed boundary edges of a boundary-set delta (hole-delta
/// probes). Up to 8 samples of each side.
#[cfg(feature = "instrumentation")]
fn dump_boundary_delta(added: &[(HalfedgeId, HalfedgeId)], removed: &[(HalfedgeId, HalfedgeId)]) {
    if !added.is_empty() {
        eprintln!("  added {}:", added.len());
        for (a, b) in added.iter().take(8) {
            eprintln!("    edge ({a:?}, {b:?})");
        }
    }
    if !removed.is_empty() {
        eprintln!("  removed {}:", removed.len());
        for (a, b) in removed.iter().take(8) {
            eprintln!("    edge ({a:?}, {b:?})");
        }
    }
}

/// Prints the face-death ledger (most recent last).
#[cfg(feature = "instrumentation")]
pub(crate) fn dump_face_death_ledger() {
    if let Ok(ledger) = FACE_DEATH_LEDGER.lock() {
        for (face_id, op) in ledger.iter() {
            eprintln!("    face {face_id:?} removed by '{op}'");
        }
    }
}

/// Called when `collapse_until_edges_above_min_length`'s neighborhood check picks a
/// dead halfedge id (inserted via `he.twin` of a live halfedge without a liveness
/// check on the twin). Scans the neighborhood for the twin violator — the live
/// halfedge whose `twin` references the dead id — and dumps it.
#[cfg(feature = "instrumentation")]
#[inline]
pub(crate) fn report_dead_halfedge_in_collapse_check(mesh_graph: &MeshGraph, dead_id: HalfedgeId) {
    if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_none() {
        return;
    }

    static REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
    if REPORTED.set(()).is_err() {
        return;
    }
    mark_integrity_violation();
    eprintln!("DEAD ID IN COLLAPSE CHECK: {dead_id:?} was inserted via a live halfedge's twin");
    for (he_id, he) in &mesh_graph.halfedges {
        if he.twin == Some(dead_id) {
            let face_alive = he.face.is_some_and(|f| mesh_graph.faces.contains_key(f));
            eprintln!(
                "  violator {he_id:?}: face={:?} (alive={face_alive}) next={:?} twin={:?} end={:?}",
                he.face, he.next, he.twin, he.end_vertex
            );
        }
    }
    eprintln!("{}", std::backtrace::Backtrace::force_capture());
    eprintln!("recent face deaths (oldest first):");
    dump_face_death_ledger();
    state_history_dump(
        "dead_id_in_collapse_check",
        Some(mesh_graph),
        Some(&dead_id),
    );
}

// -- state history (clone ring + resume) ----------------------------------------
//
// Debug support for the layer-4 corruption hunt, compiled only with the
// `instrumentation` feature and gated at runtime on `MESH_GRAPH_DANGLING_CHECK=1`:
// after every op end that passes the chain-integrity validator a clone of the mesh
// is pushed onto a small ring (default 10 entries). When a corruption probe fires,
// the ring is written to disk so the op that introduced the corruption can be
// diagnosed from the states leading up to it, and the run can be resumed from any
// of them via `MeshGraph::load_state`.
//
// Env vars:
//   MESH_GRAPH_DANGLING_CHECK        – enable the whole hunt instrumentation
//   MESH_GRAPH_STATE_HISTORY_LEN     – ring capacity (default 10)
//   MESH_GRAPH_STATE_DUMP_DIR        – dump directory (default `mesh_graph_state_dump_<pid>`)
//   MESH_GRAPH_STATE_DUMP_AT_POS     – dump the ring (once) when the replay position
//                                      reaches this value (capture-on-demand for tests)
//   MESH_GRAPH_HOLE_CHECK            – boundary-delta probe: each probed op must leave
//                                      the boundary edge set unchanged vs. its own entry
//                                      (per-op snapshot by `probe_chain_begin`); closed
//                                      regions mark violations, open (punch-rim) regions
//                                      only report
//
// The replay position is a *step index*: the host reports one step per mesh-graph
// topology op call (collapse / subdivide / individual merge_one_ring) via
// [`MeshGraph::set_replay_position`], so ring snapshots map 1:1 to the host's
// operation journal and a run can be resumed from any dumped state by replaying
// the remaining journal steps (freestyle-sculpt: `MESH_GRAPH_RESUME_STATE` +
// `MESH_GRAPH_RESUME_INDEX`).

/// A short trace of the structural re-wiring events (flips, flap removals, fresh
/// boundary pairings, ...) that led up to a corruption report. Each entry records
/// the event kind plus the halfedges/vertices it touched, so the corruption report
/// can show which writer produced the violated ids instead of guessing from the
/// mesh diff. Ring of the last [`OP_TRACE_CAP`] events; printed by the corruption
/// report on demand.
#[cfg(feature = "instrumentation")]
pub(crate) static OP_TRACE: std::sync::Mutex<std::collections::VecDeque<String>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

#[cfg(feature = "instrumentation")]
pub(crate) const OP_TRACE_CAP: usize = 2000;

/// Whether the op trace records events. Enabled at runtime with
/// `MESH_GRAPH_TRACE=1` (on top of `MESH_GRAPH_DANGLING_CHECK`).
#[cfg(feature = "instrumentation")]
pub(crate) fn op_trace_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_some()
            && std::env::var_os("MESH_GRAPH_TRACE").is_some()
    })
}

/// Records one structural re-wiring event into [`OP_TRACE`] (no-op when the hunt
/// instrumentation is disabled at runtime, including the `format!` itself).
#[cfg(feature = "instrumentation")]
#[macro_export]
macro_rules! record_op_trace {
    ($($arg:tt)*) => {
        if $crate::op_trace_enabled() {
            $crate::record_op_trace_impl(format_args!($($arg)*).to_string());
        }
    };
}

#[cfg(feature = "instrumentation")]
#[inline]
pub(crate) fn record_op_trace_impl(event: String) {
    if let Ok(mut trace) = OP_TRACE.lock() {
        trace.push_back(event);
        while trace.len() > OP_TRACE_CAP {
            trace.pop_front();
        }
    }
}

/// The replay position (input-log entry index) reported by the host application
/// through [`MeshGraph::set_replay_position`]. Stored with every state snapshot so
/// a dumped state can be resumed at the exact spot it was captured.
#[cfg(feature = "instrumentation")]
static REPLAY_POSITION: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Reports the host's current replay position (input-log entry index). The host
/// calls [`MeshGraph::set_replay_position`] for every consumed entry; state
/// snapshots record it so a dumped state can be resumed at the exact spot it was
/// captured.
#[cfg(feature = "instrumentation")]
pub fn set_replay_position(pos: u64) {
    REPLAY_POSITION.store(pos, std::sync::atomic::Ordering::Relaxed);
}

#[cfg(feature = "instrumentation")]
fn replay_position() -> u64 {
    REPLAY_POSITION.load(std::sync::atomic::Ordering::Relaxed)
}

/// Set on the first integrity-violation report in this process, so hosts and
/// tests can assert that a replay of a dumped state stayed clean.
#[cfg(feature = "instrumentation")]
static INTEGRITY_VIOLATION: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Marks the first detected integrity violation (see [`integrity_violation_reported`]).
#[cfg(feature = "instrumentation")]
pub(crate) fn mark_integrity_violation() {
    INTEGRITY_VIOLATION.store(true, std::sync::atomic::Ordering::Relaxed);
}

/// Returns `true` once any corruption report has fired in this process. Hunt
/// instrumentation only; always `false` without the `instrumentation` feature.
#[cfg(feature = "instrumentation")]
pub fn integrity_violation_reported() -> bool {
    INTEGRITY_VIOLATION.load(std::sync::atomic::Ordering::Relaxed)
}

/// One verified mesh state: the mesh clone plus the position/op it was captured at.
#[cfg(feature = "instrumentation")]
struct StateSnapshot {
    pos: u64,
    op: &'static str,
    mesh: MeshGraph,
}

/// The ring of the most recent verified states (oldest first).
#[cfg(feature = "instrumentation")]
pub(crate) static STATE_RING: std::sync::Mutex<std::collections::VecDeque<StateSnapshot>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

#[cfg(feature = "instrumentation")]
pub(crate) const STATE_RING_DEFAULT_CAP: usize = 10;

/// Set once the state history has been dumped, so hosts can write sidecar data
/// (e.g. an operation journal) into the same directory.
#[cfg(feature = "instrumentation")]
static STATE_DUMP_DIR: std::sync::OnceLock<std::path::PathBuf> = std::sync::OnceLock::new();

/// The directory the state history was dumped to (if a dump happened in this
/// process). Hosts can use it to write sidecar files (like an operation journal)
/// next to the dumped states.
#[cfg(feature = "instrumentation")]
pub fn state_dump_dir() -> Option<std::path::PathBuf> {
    STATE_DUMP_DIR.get().cloned()
}

#[cfg(feature = "instrumentation")]
fn state_ring_cap() -> usize {
    std::env::var_os("MESH_GRAPH_STATE_HISTORY_LEN")
        .and_then(|s| s.to_str().and_then(|s| s.parse::<usize>().ok()))
        // 0 disables the ring entirely (faster hunt runs that only probe).
        .unwrap_or(STATE_RING_DEFAULT_CAP)
}

/// Pushes a clone of the current mesh onto the state-history ring. Only called at
/// op ends that passed the chain-integrity validator. Env-gated: no-op when the
/// hunt instrumentation is disabled.
#[cfg(feature = "instrumentation")]
#[inline]
pub(crate) fn state_history_push(mesh: &MeshGraph, op: &'static str) {
    if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_none() {
        return;
    }

    let snapshot = StateSnapshot {
        pos: replay_position(),
        op,
        mesh: mesh.clone(),
    };
    if let Ok(mut ring) = STATE_RING.lock() {
        let cap = state_ring_cap();
        if cap == 0 {
            ring.clear();
            return;
        }
        ring.push_back(snapshot);
        while ring.len() > cap {
            ring.pop_front();
        }
    }

    // Capture-on-demand: dump the ring (once) once the boom position is reached.
    if let Some(target) = std::env::var_os("MESH_GRAPH_STATE_DUMP_AT_POS")
        && let Some(target) = target.to_str().and_then(|s| s.parse::<u64>().ok())
        && replay_position() >= target
    {
        state_history_dump("at_position", Some(mesh), None);
    }
}

/// Writes the state-history ring to disk (plus an optional `current` state), so
/// the states leading up to a corruption event can be inspected/resumed from.
/// Fires at most once per process. Needs the `serde` feature (JSON state files).
#[cfg(feature = "instrumentation")]
pub(crate) fn state_history_dump(
    reason: &str,
    current: Option<&MeshGraph>,
    context: Option<&dyn std::fmt::Debug>,
) {
    if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_none() {
        return;
    }

    static DUMPED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
    if DUMPED.set(()).is_err() {
        return;
    }

    let dir = std::env::var_os("MESH_GRAPH_STATE_DUMP_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| {
            std::path::PathBuf::from(format!("mesh_graph_state_dump_{}", std::process::id()))
        });
    if let Err(e) = std::fs::create_dir_all(&dir) {
        eprintln!("state dump: could not create {}: {e:?}", dir.display());
        return;
    }
    // Advertise the dump directory so hosts can write sidecars (e.g. the op
    // journal) into it.
    let _ = STATE_DUMP_DIR.set(dir.clone());

    let mut meta = String::new();
    meta.push_str(&format!(
        "reason: {reason}\ncurrent replay position: {}\n",
        replay_position()
    ));
    if let Some(context) = context {
        meta.push_str(&format!("context: {context:?}\n"));
    }

    if let Ok(ring) = STATE_RING.lock() {
        meta.push_str("ring entries (oldest first):\n");
        for (i, snap) in ring.iter().enumerate() {
            meta.push_str(&format!(
                "  state_{i:02}: pos={} op={}\n",
                snap.pos, snap.op
            ));
        }
    }

    if let Some(current) = current {
        let path = dir.join("current.json");
        if let Err(e) = current.save_state(&path) {
            eprintln!("state dump: could not write {}: {e:?}", path.display());
        }
        meta.push_str(&format!(
            "current.json: current broken state (pos {})\n",
            replay_position()
        ));
    }

    if let Ok(ring) = STATE_RING.lock() {
        for (i, snap) in ring.iter().enumerate() {
            let path = dir.join(format!("state_{i:02}_pos_{:06}_{}.json", snap.pos, snap.op));
            if let Err(e) = snap.mesh.save_state(&path) {
                eprintln!("state dump: could not write {}: {e:?}", path.display());
            }
        }
    }

    if let Err(e) = std::fs::write(dir.join("meta.txt"), meta) {
        eprintln!("state dump: could not write meta.txt: {e:?}");
    }
    eprintln!("state history dumped to {}", dir.display());
}

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

    /// Pairs a surviving halfedge with a freshly created boundary halfedge so a
    /// halfedge is never left twinless after its partner is removed or re-paired
    /// elsewhere (every halfedge must have a twin in a valid state; boundary edges
    /// are represented as a pair of a face member and a detached half).
    ///
    /// `survivor_start_v` is the survivor's start vertex (derived by the caller,
    /// since the survivor's own twin may already be gone). The new halfedge claims
    /// no face and is registered in `outgoing_halfedges` under its start vertex
    /// (= the survivor's end vertex). Returns the new boundary halfedge.
    #[instrument(skip(self))]
    fn pair_with_fresh_boundary_half(
        &mut self,
        survivor_id: HalfedgeId,
        survivor_start_v: VertexId,
    ) -> Option<HalfedgeId> {
        let survivor = self
            .halfedges
            .get(survivor_id)
            .or_else(error_none!("survivor he not found"))?;
        let survivor_end = survivor.end_vertex;
        // `add_halfedge` already registers `boundary_id` in `outgoing_halfedges`
        // under its start vertex (= `survivor_end`), so do not push it again here.
        let boundary_id = self.add_halfedge(survivor_end, survivor_start_v)?;
        // just checked above that the survivor exists
        self.halfedges[survivor_id].twin = Some(boundary_id);
        // just added above
        self.halfedges[boundary_id].twin = Some(survivor_id);

        #[cfg(feature = "instrumentation")]
        crate::record_op_trace!(
            "fresh boundary {boundary_id:?} ({survivor_end:?}->{survivor_start_v:?}) paired with survivor {survivor_id:?}"
        );

        Some(boundary_id)
    }

    /// Repairs a vertex's `outgoing_halfedge` seed when it points at a halfedge that
    /// no longer exists. Ring traversals (`one_ring`, faces, ...) start from this seed,
    /// so a dead seed makes them yield removed ids. Replaces it with any live outgoing
    /// halfedge of the vertex, or `None` if the vertex has become isolated. A live seed
    /// is left untouched to keep ring iteration order stable in the common case.
    fn reseed_outgoing_if_dead(&mut self, vertex_id: VertexId) {
        let Some(vertex) = self.vertices.get(vertex_id) else {
            return;
        };
        if vertex
            .outgoing_halfedge
            .is_some_and(|he| self.halfedges.contains_key(he))
        {
            return;
        }
        let new_seed = self.outgoing_halfedges.get(vertex_id).and_then(|list| {
            list.iter()
                .copied()
                .find(|he| self.halfedges.contains_key(*he))
        });
        if let Some(v) = self.vertices.get_mut(vertex_id) {
            v.outgoing_halfedge = new_seed;
        }
    }

    /// Debug probe for the layer-4 corruption hunt: with `MESH_GRAPH_DANGLING_CHECK=1`
    /// reports once (per process) when a halfedge removal takes a halfedge that still
    /// belongs to a *live* face — a face that is not being dismantled by the same call
    /// (`op` is not `remove_face_tail` / `remove_halfedge_face`) — and whose other
    /// members survive. Such a removal breaks the face chain and corrupts the mesh.
    ///
    /// Pure instrumentation: it never mutates the mesh. Removal sites used to funnel
    /// through `clear_twins_to`, which nulled the surviving partner's `twin` before
    /// the `halfedges.remove(...)`. Re-pairing is now done locally at each removal
    /// site, so no `.twin = None` write exists in the codebase anymore: every surviving
    /// halfedge is re-paired (fresh boundary half, partner swap) or removed in the same
    /// batch as its partner before its operation terminates.
    #[cfg(feature = "instrumentation")]
    pub(crate) fn probe_live_face_removal(&self, removed_ids: &[HalfedgeId], op: &str) {
        if removed_ids.is_empty() {
            return;
        }

        static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        static REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
        if *ENABLED.get_or_init(|| std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_some())
            && !matches!(op, "remove_face_tail" | "remove_halfedge_face")
        {
            for id in removed_ids {
                if let Some(he) = self.halfedges.get(*id)
                    && let Some(face_id) = he.face
                    && self.faces.contains_key(face_id)
                {
                    let other_members: Vec<HalfedgeId> = self
                        .halfedges
                        .iter()
                        .filter(|(h_id, h)| {
                            h.face == Some(face_id) && *h_id != *id && !removed_ids.contains(h_id)
                        })
                        .map(|(h_id, _)| h_id)
                        .take(4)
                        .collect();
                    if !other_members.is_empty() && REPORTED.set(()).is_ok() {
                        mark_integrity_violation();
                        eprintln!(
                            "REMOVING LIVE-FACE MEMBER {id:?} of face {face_id:?} (surviving members {other_members:?})"
                        );
                        eprintln!("{}", std::backtrace::Backtrace::force_capture());
                        state_history_dump("live_face_member_removal", Some(self), Some(&id));
                    }
                }
            }
        }
    }

    /// Debug probe for the layer-4 corruption hunt: with `MESH_GRAPH_DANGLING_CHECK=1`
    /// reports once (per process) the first operation that terminates with a broken
    /// face chain — a live halfedge whose `next` is `None`, references a removed
    /// halfedge, or references a halfedge claimed by a different (or no) face.
    ///
    /// Such a chain is what later yields dead ids into `one_ring` walks and
    /// subdivide/collapse bookkeeping (which panic on the stale SlotMap key), long
    /// after the op that actually broke the chain. Checking at op boundaries catches
    /// the writer instead of the walker.
    ///
    /// Beyond the chains, the probe also verifies the two other bookkeeping
    /// invariants against their rebuild ground truth:
    ///
    /// - **Twin invariant** (every live halfedge, incl. boundary halves, must have a
    ///   live mutual twin at op end) and
    /// - **outgoing lists**: `outgoing_halfedges[V]` must contain exactly the twins
    ///   of the live halfedges ending at `V` (what [`rebuild_outgoing_halfedges`]
    ///   produces). Missing/extra ids are corruption.
    ///
    /// Both are state-dumped on first violation. List *order* and per-vertex seed
    /// deviations have legitimate alternatives (ops may re-order lists; the rebuild
    /// keeps live seeds), so those are reported once without consuming the dump.
    ///
    /// Pure instrumentation: it never mutates the mesh. Returns `true` when the
    /// mesh passed all checks; on the first corruption the state-history ring is
    /// dumped to disk (see [`state_history_dump`]).
    #[cfg(feature = "instrumentation")]
    pub(crate) fn probe_chain_integrity(&self, op: &str) -> bool {
        if std::env::var_os("MESH_GRAPH_DANGLING_CHECK").is_none() {
            return true;
        }

        static REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();

        let mut violations: Vec<(HalfedgeId, &'static str)> = Vec::new();
        let mut dead_face_siblings: Vec<FaceId> = Vec::new();
        for (he_id, he) in &self.halfedges {
            let Some(face_id) = he.face else {
                continue;
            };

            let verdict = if !self.faces.contains_key(face_id) {
                if !dead_face_siblings.contains(&face_id) {
                    dead_face_siblings.push(face_id);
                }
                Some("member of a removed face")
            } else {
                match he.next {
                    None => Some("member of a live face with next=None"),
                    Some(next_id) => match self.halfedges.get(next_id) {
                        None => Some("next references a removed halfedge"),
                        Some(next_he) if next_he.face != Some(face_id) => {
                            Some("next references a halfedge of another/no face")
                        }
                        Some(_) => None,
                    },
                }
            };

            if let Some(reason) = verdict {
                violations.push((he_id, reason));
                if violations.len() >= 12 {
                    break;
                }
            }
        }

        // Family-vs-chain fork check: every halfedge that claims a live face (its
        // member family) must also be reachable through the face's `next` chain.
        // A face whose family contains halfedges outside its chain is the face-steal
        // residue: `add_face` re-filed the stray while the face's own chain re-uses
        // (or lost) it elsewhere.
        let mut fork: Option<(FaceId, Vec<HalfedgeId>, Vec<HalfedgeId>)> = None;
        let mut family_by_face: hashbrown::HashMap<FaceId, Vec<HalfedgeId>> =
            hashbrown::HashMap::new();
        for (h_id, h) in &self.halfedges {
            if let Some(f) = h.face {
                family_by_face.entry(f).or_default().push(h_id);
            }
        }
        for (face_id, face) in &self.faces {
            let Some(family) = family_by_face.get(&face_id) else {
                continue;
            };
            if family.len() < 3 {
                // Not a proper triangle face; the other verdicts cover the broken cases.
                continue;
            }
            let mut chain = Vec::with_capacity(family.len());
            let mut cur = Some(face.halfedge);
            let mut steps = 0;
            while let Some(he_id) = cur {
                if !self.halfedges.contains_key(he_id) || chain.len() > family.len() + 2 {
                    break;
                }
                if chain.contains(&he_id) {
                    break;
                }
                chain.push(he_id);
                cur = self.halfedges[he_id].next;
                steps += 1;
                if steps > 16 {
                    break;
                }
            }
            if chain.len() != family.len() || family.iter().any(|h_id| !chain.contains(h_id)) {
                fork = Some((face_id, family.clone(), chain));
                break;
            }
        }

        // --- Twin invariant + outgoing-halfedge ground truth ---
        // `rebuild_outgoing_halfedges` is the definite ground truth for the
        // per-vertex lists: `outgoing_halfedges[V]` must contain exactly the twins
        // of the live halfedges ending at V, in halfedge-iteration order. This
        // sweep checks every live halfedge (incl. boundary halves, which the chain
        // check above skips): a halfedge without a live mutual twin, or a list with
        // missing/extra ids at op end, is corruption.
        let fmt_ids = |ids: &[HalfedgeId]| -> String {
            if ids.len() <= 8 {
                format!("{ids:?}")
            } else {
                format!("{:?}... ({} ids)", &ids[..8], ids.len())
            }
        };
        let mut twin_problems: Vec<String> = Vec::new();
        let mut membership_problems: Vec<String> = Vec::new();
        // Order deviations are expected to be pervasive (ops re-order lists), so
        // only a counter plus the first sample is kept.
        let mut order_deviation_count: usize = 0;
        let mut first_order_sample: Option<String> = None;
        // Reusable scratch buffers: the probe runs at every op end (~1600x per
        // log replay), so allocations are retained across calls instead of
        // churning the allocator (which showed up as minutes of sys time).
        struct OutScratch {
            expected: hashbrown::HashMap<VertexId, Vec<HalfedgeId>>,
            counts: hashbrown::HashMap<HalfedgeId, usize>,
        }
        static OUT_SCRATCH: std::sync::Mutex<Option<OutScratch>> = std::sync::Mutex::new(None);
        let mut scratch = OUT_SCRATCH.lock().unwrap();
        let scratch = scratch.get_or_insert_with(|| OutScratch {
            expected: hashbrown::HashMap::new(),
            counts: hashbrown::HashMap::new(),
        });
        for list in scratch.expected.values_mut() {
            list.clear();
        }
        scratch.expected.clear();
        scratch.counts.clear();

        for (he_id, he) in &self.halfedges {
            match he.twin {
                None => twin_problems.push(format!("halfedge {he_id:?} has twin=None")),
                Some(twin_id) => {
                    if !self.halfedges.contains_key(twin_id) {
                        twin_problems.push(format!(
                            "halfedge {he_id:?} has twin {twin_id:?} which is removed"
                        ));
                    } else if self.halfedges[twin_id].twin != Some(he_id) {
                        twin_problems.push(format!(
                            "halfedge {he_id:?} has twin {twin_id:?} which does not point back"
                        ));
                    }
                }
            }
            if let Some(twin_id) = he.twin {
                scratch
                    .expected
                    .entry(he.end_vertex)
                    .or_default()
                    .push(twin_id);
            }
        }

        for (v_id, expected) in scratch.expected.iter() {
            let actual: &[HalfedgeId] = self
                .outgoing_halfedges
                .get(*v_id)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            // Multiset difference in O(|actual| + |expected|) via a count map.
            scratch.counts.clear();
            for &a in actual {
                *scratch.counts.entry(a).or_default() += 1;
            }
            let mut missing: Vec<HalfedgeId> = Vec::new();
            for &exp in expected {
                match scratch.counts.get_mut(&exp) {
                    Some(c) if *c > 0 => *c -= 1,
                    _ => missing.push(exp),
                }
            }
            let mut extra: Vec<HalfedgeId> = Vec::new();
            for &a in actual {
                if scratch.counts.get(&a) != Some(&0) {
                    extra.push(a);
                }
            }
            if !missing.is_empty() || !extra.is_empty() {
                let mut msg = format!("vertex {v_id:?}: outgoing deviates from ground truth");
                if !missing.is_empty() {
                    msg += &format!(", missing {}", fmt_ids(&missing));
                }
                if !extra.is_empty() {
                    msg += &format!(", extra {}", fmt_ids(&extra));
                }
                membership_problems.push(msg);
            } else if actual != expected.as_slice() {
                order_deviation_count += 1;
                if first_order_sample.is_none() {
                    first_order_sample = Some(format!(
                        "vertex {v_id:?}: outgoing order differs from rebuild order"
                    ));
                }
            }
        }
        // Vertices holding list entries although no live halfedge ends at them.
        for (v_id, actual) in &self.outgoing_halfedges {
            if !scratch.expected.contains_key(&v_id) && !actual.is_empty() {
                membership_problems.push(format!(
                    "vertex {v_id:?}: outgoing {} but no live halfedge ends at it",
                    fmt_ids(actual)
                ));
            }
        }

        // Per-vertex seed normalization, simulated from `rebuild_outgoing_halfedges`:
        // a live seed is kept, a dead seed is replaced with the first list entry.
        let mut seed_deviations: Vec<String> = Vec::new();
        for (v_id, vertex) in &self.vertices {
            let stored = vertex.outgoing_halfedge;
            let rebuilt_seed = stored
                .filter(|he| self.halfedges.contains_key(*he))
                .or_else(|| scratch.expected.get(&v_id).and_then(|l| l.first().copied()));
            if stored != rebuilt_seed {
                seed_deviations.push(format!(
                    "vertex {v_id:?}: seed {stored:?} != rebuilt {rebuilt_seed:?}"
                ));
            }
        }

        let clean =
            violations.is_empty() && twin_problems.is_empty() && membership_problems.is_empty();

        if !clean && REPORTED.set(()).is_ok() {
            mark_integrity_violation();
            eprintln!(
                "CHAIN CORRUPTION detected at end of op '{op}' ({} violations shown):",
                violations.len()
            );
            for (he_id, reason) in violations {
                let detail = self.halfedges.get(he_id).map(|he| {
                    format!(
                        "face={:?} next={:?} twin={:?} end={:?}",
                        he.face, he.next, he.twin, he.end_vertex
                    )
                });
                eprintln!("  halfedge {he_id:?}: {reason}; {detail:?}");
                // Neighborhood dump: the halfedge's next target, twin, and the
                // start vertex's outgoing star, so the ghost's surroundings are
                // visible (which live faces/edges it connects to).
                if let Some(he) = self.halfedges.get(he_id) {
                    if let Some(next_id) = he.next
                        && let Some(next_he) = self.halfedges.get(next_id)
                    {
                        eprintln!(
                            "    next {next_id:?}: face={:?} next={:?} twin={:?} end={:?}",
                            next_he.face, next_he.next, next_he.twin, next_he.end_vertex
                        );
                    }
                    if let Some(twin_id) = he.twin
                        && let Some(twin_he) = self.halfedges.get(twin_id)
                    {
                        eprintln!(
                            "    twin {twin_id:?}: face={:?} next={:?} twin={:?} end={:?}",
                            twin_he.face, twin_he.next, twin_he.twin, twin_he.end_vertex
                        );
                    }
                    if let Some(start_v) = he.start_vertex(self) {
                        let out: Vec<HalfedgeId> = self
                            .outgoing_halfedges
                            .get(start_v)
                            .map(|l| l.iter().copied().take(6).collect())
                            .unwrap_or_default();
                        let out_desc: Vec<String> = out
                            .iter()
                            .filter_map(|id| {
                                self.halfedges
                                    .get(*id)
                                    .map(|h| format!("{id:?}(face={:?},next={:?})", h.face, h.next))
                            })
                            .collect();
                        eprintln!("    start vertex {start_v:?} outgoing: {out_desc:?}");
                    }
                }
            }
            // For halfedges that claim a removed face, print the other live halfedges
            // claiming the same dead face (the family that escaped the removal).
            for dead_face_id in &dead_face_siblings {
                let family: Vec<HalfedgeId> = self
                    .halfedges
                    .iter()
                    .filter(|(_, h)| h.face == Some(*dead_face_id))
                    .map(|(h_id, _)| h_id)
                    .collect();
                eprintln!("  halfedges claiming removed face {dead_face_id:?}: {family:?}");
            }
            eprintln!("{}", std::backtrace::Backtrace::force_capture());
            eprintln!("recent face deaths (oldest first):");
            dump_face_death_ledger();
            if let Some((fork_face, fork_family, fork_chain)) = fork {
                eprintln!("family-vs-chain for face {fork_face:?}:");
                eprintln!("  chain  (walked): {fork_chain:?}");
                eprintln!("  family (face field): {fork_family:?}");
            }
            for problem in &twin_problems {
                eprintln!("TWIN: {problem}");
            }
            for problem in membership_problems.iter().take(12) {
                eprintln!("OUTGOING: {problem}");
            }
            if let Ok(trace) = OP_TRACE.lock() {
                eprintln!("op trace (oldest first):");
                for event in trace.iter() {
                    eprintln!("  {event}");
                }
            }
            state_history_dump("chain_integrity", Some(self), Some(&op));
        }

        // Hole-delta detector (`MESH_GRAPH_HOLE_CHECK=1` on top of
        // `MESH_GRAPH_DANGLING_CHECK`): each probed op (collapse/subdivide/merge/...)
        // must leave the boundary edge set exactly as it was at its own entry (see
        // [`crate::probe_chain_begin`]).
        //
        // Two levels, split by the op's own entry boundary: an op that starts on a
        // closed region (weld runs) must stay closed — any boundary change marks an
        // integrity violation. An op that starts next to a punch-hole rim may
        // legitimately swap rim edges (cleanup collapses can consume a rim edge and
        // re-pair its twin with the new fan edge, a 1:1 boundary swap) — that is
        // reported informationally only, so the integrity flag can't be contaminated
        // by expected rim evolution; a real defect there would additionally trip the
        // chain/twin probes. `remove_face`-family ops are not probed, so the
        // intentional punch itself is never blamed. Reported once per process,
        // separately from the chain corruption report so the two signals don't mask
        // each other.
        if std::env::var_os("MESH_GRAPH_HOLE_CHECK").is_some() {
            static HOLE_REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
            static RIM_REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
            let current = self.boundary_edge_set();
            let begin = OP_BOUNDARY.with(|b| b.borrow().clone());
            if let Some(begin) = begin {
                let added: Vec<(HalfedgeId, HalfedgeId)> =
                    current.difference(&begin).copied().collect();
                let removed: Vec<(HalfedgeId, HalfedgeId)> =
                    begin.difference(&current).copied().collect();
                if !added.is_empty() || !removed.is_empty() {
                    let boundary_count =
                        self.halfedges.values().filter(|h| h.face.is_none()).count();
                    if begin.is_empty() {
                        // The op started on a closed region: any boundary change is a
                        // defect.
                        if HOLE_REPORTED.set(()).is_ok() {
                            mark_integrity_violation();
                            eprintln!(
                                "HOLE DELTA: op '{op}' changed the boundary edge set of a closed region (now {boundary_count} boundary halfedges):"
                            );
                            dump_boundary_delta(&added, &removed);
                            eprintln!("{}", std::backtrace::Backtrace::force_capture());
                            if let Ok(trace) = OP_TRACE.lock() {
                                eprintln!("op trace (oldest first):");
                                for event in trace.iter() {
                                    eprintln!("  {event}");
                                }
                            }
                            state_history_dump("hole", Some(self), Some(&op));
                        }
                    } else if RIM_REPORTED.set(()).is_ok() {
                        // The op started next to a punch rim: boundary swaps are
                        // expected during punch cleanup; only informational.
                        eprintln!(
                            "RIM DELTA: op '{op}' changed the boundary edge set of an open region (now {boundary_count} boundary halfedges) — expected during punch cleanup:"
                        );
                        dump_boundary_delta(&added, &removed);
                    }
                }
            }
        }

        // Possibly-legitimate alternatives to the rebuild ground truth (list order,
        // seed choice): reported once per process with counts, without consuming
        // the once-per-process corruption dump above.
        if order_deviation_count > 0 {
            static ORDER_REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
            if ORDER_REPORTED.set(()).is_ok() {
                eprintln!(
                    "OUTGOING ORDER deviates from rebuild order at {order_deviation_count} vertices (first: {})",
                    first_order_sample.as_deref().unwrap_or("")
                );
            }
        }
        if !seed_deviations.is_empty() {
            static SEED_REPORTED: std::sync::OnceLock<()> = std::sync::OnceLock::new();
            if SEED_REPORTED.set(()).is_ok() {
                eprintln!(
                    "SEED deviates from rebuild at op '{op}' at {} vertices (first: {})",
                    seed_deviations.len(),
                    seed_deviations[0]
                );
            }
        }

        clean
    }

    /// Undirected boundary edge set: the normalized `(min, max)` pairs of the
    /// twin couple of every live halfedge with `face = None`. Two meshes with the
    /// same open edges produce equal sets regardless of fan order, so a
    /// before/after comparison detects boundary changes without being sensitive
    /// to halfedge iteration order. Only used by the hole-delta probe.
    #[cfg(feature = "instrumentation")]
    fn boundary_edge_set(&self) -> hashbrown::HashSet<(HalfedgeId, HalfedgeId)> {
        let mut edges = hashbrown::HashSet::new();
        for (he_id, he) in &self.halfedges {
            if he.face.is_none()
                && let Some(twin_id) = he.twin
            {
                edges.insert(if twin_id < he_id {
                    (twin_id, he_id)
                } else {
                    (he_id, twin_id)
                });
            }
        }
        edges
    }

    /// Ground-truth rebuild of a single vertex's outgoing list: the halfedges
    /// whose twins end at the vertex (the inverse of `rebuild_outgoing_halfedges`,
    /// which derives the lists from the halfedge iteration). Unlike a seed-based
    /// ring walk, this is immune to a dead or stale seed and to temporarily
    /// detached (face-less) halfedges, so it never shrinks a vertex's list below
    /// its true star.
    ///
    /// The seed is refreshed with `rebuild_outgoing_halfedges` semantics: a live
    /// seed is kept, otherwise the first list entry is used.
    pub fn rebuild_vertex_outgoing_list(&mut self, vertex_id: VertexId) {
        let mut list: Vec<HalfedgeId> = Vec::new();
        for (_, he) in &self.halfedges {
            if he.end_vertex == vertex_id
                && let Some(twin_id) = he.twin
                && self.halfedges.contains_key(twin_id)
            {
                list.push(twin_id);
            }
        }

        if let Some(entry) = self.outgoing_halfedges.get_mut(vertex_id) {
            *entry = list;
        }

        if let Some(vertex) = self.vertices.get_mut(vertex_id)
            && !vertex
                .outgoing_halfedge
                .is_some_and(|he| self.halfedges.contains_key(he))
        {
            vertex.outgoing_halfedge = self
                .outgoing_halfedges
                .get(vertex_id)
                .and_then(|l| l.first().copied());
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
                self.outgoing_halfedges
                    .get(v_id)
                    .and_then(|list| list.first().copied())
            });
        }
    }
}
