//! Rim tooling for `merge_vertices_one_rings`: the view of the boundary that
//! the op's removals *actually* open.
//!
//! Test-only for now; the rim-first rebuild promotes this to production use.
//! Two definitions here are load-bearing and were both wrong in the obvious
//! formulation:
//!
//! - [`opened_rim`] is `boundary(after) \ boundary(before)`, **not** the twins
//!   of the removed halfedges. `remove_face` (remove.rs, the `else` branch)
//!   detaches the dominant rim members in place (`face = None`, `next = None`,
//!   id kept) and only *returns* the halves it actually frees. The removal
//!   lists therefore miss exactly the dense part of the rim.
//! - [`next_rim_half`] scans the raw `outgoing_halfedges` list of the end
//!   vertex. Rotation-based walks cannot traverse a boundary:
//!   `cw_rotated_neighbour` is `twin.next`, boundary halves have `next = None`,
//!   and the `outgoing_halfedges` iterator is built on that rotation, so it
//!   stops one step short of the next rim half.

use hashbrown::HashSet;

use crate::{HalfedgeId, MeshGraph};

/// All live halfedges without a face: the current boundary.
pub(crate) fn boundary_halves(mesh: &MeshGraph) -> HashSet<HalfedgeId> {
    mesh.halfedges
        .iter()
        .filter_map(|(he_id, he)| he.face.is_none().then_some(he_id))
        .collect()
}

/// The boundary halves the removals opened: `boundary(after) \ boundary(before)`.
///
/// Snapshot `boundary_before` immediately before a removal batch. No allocation
/// happens inside a removal, so ids are stable across the snapshot; the
/// generational ids of the slotmaps keep freed and reused slots apart.
pub(crate) fn opened_rim(
    mesh: &MeshGraph,
    boundary_before: &HashSet<HalfedgeId>,
) -> HashSet<HalfedgeId> {
    boundary_halves(mesh)
        .into_iter()
        .filter(|he_id| !boundary_before.contains(he_id))
        .collect()
}

/// One step of a rim walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RimStep {
    /// The next rim half, on the rim the removals opened.
    Continue(HalfedgeId),
    /// The chain ends on a halfedge that was already boundary before the
    /// removals: a pre-existing open border.
    ChainEnd,
    /// The walk cannot be continued: missing or cyclic connectivity.
    Isolated,
}

/// The next rim half after `b` (`u -> v`, no face).
///
/// The walk turns at `v` through the face fan: start from `twin(b)` (an
/// outgoing halfedge at `v`), step counter-clockwise
/// ([`Halfedge::ccw_rotated_neighbour`]) while the current halfedge has a
/// face, and stop at the first face-less halfedge. That is the boundary
/// continuation of `b`.
///
/// Do **not** scan the raw `outgoing_halfedges` list for "the unique face-less
/// half": after removing the two stars, a vertex shared by both rings sits on
/// two boundary loops at once (a pinch), so it has more than one face-less
/// outgoing halfedge - which one continues `b` depends on which fan `b`'s twin
/// is in, and only the rotation knows.
///
/// Also do not rotate clockwise from `b` or `twin(b)`:
/// `cw_rotated_neighbour` is `twin.next`, and `next` is `None` for a boundary
/// halfedge, so that walk stops immediately.
pub(crate) fn next_rim_half(mesh: &MeshGraph, b: HalfedgeId, rim: &HashSet<HalfedgeId>) -> RimStep {
    let Some(b_he) = mesh.halfedges.get(b) else {
        return RimStep::Isolated;
    };
    let end_v_id = b_he.end_vertex;
    let Some(mut x) = b_he.twin else {
        return RimStep::Isolated;
    };

    // The walk visits at most one outgoing halfedge per face incident to `v`.
    let cap = mesh
        .outgoing_halfedges
        .get(end_v_id)
        .map_or(0, |out_hes| out_hes.len())
        + 2;

    for _ in 0..cap {
        let Some(he) = mesh.halfedges.get(x) else {
            return RimStep::Isolated;
        };
        if he.face.is_none() {
            if x == b {
                return RimStep::Isolated;
            }
            return if rim.contains(&x) {
                RimStep::Continue(x)
            } else {
                RimStep::ChainEnd
            };
        }
        let Some(next) = he.ccw_rotated_neighbour(mesh) else {
            return RimStep::Isolated;
        };
        if next == x {
            return RimStep::Isolated;
        }
        x = next;
    }

    RimStep::Isolated
}
