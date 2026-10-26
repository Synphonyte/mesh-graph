//! Rim tooling for `merge_vertices_one_rings`: the view of the boundary that
//! the op's removals *actually* open.
//!
//! The op computes its rim with [`opened_rim_around`] and the rim-first planner
//! walks it with [`next_rim_half`]. One definition here is load-bearing and was
//! wrong in the obvious formulation:
//!
//! - [`opened_rim`] is `boundary(after) \ boundary(before)`, **not** the twins
//!   of the removed halfedges. `remove_face` (remove.rs, the `else` branch)
//!   detaches the dominant rim members in place (`face = None`, `next = None`,
//!   id kept) and only *returns* the halves it actually frees. The removal
//!   lists therefore miss exactly the dense part of the rim.
//!
//! [`next_rim_half`] deliberately rotates counter-clockwise through the face fan
//! from `twin(b)` rather than scanning the raw `outgoing_halfedges` list: at a
//! pinched vertex shared by both rings more than one face-less outgoing half
//! exists, and only the rotation knows which one continues `b`. Clockwise
//! rotation cannot be used either (`cw_rotated_neighbour` is `twin.next`, and a
//! boundary half's `next` is `None`). See its own doc comment.

use crate::hash::HashSet;

use crate::ops::collapse::{BoundaryAtEntry, VertexRenames, edge_key};
use crate::{Halfedge, HalfedgeId, MeshGraph, VertexId};

/// All live halfedges without a face: the current boundary.
///
/// A scan of the whole mesh; the op itself uses [`boundary_halves_around`].
#[cfg(test)]
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
#[cfg(test)]
pub(crate) fn opened_rim(
    mesh: &MeshGraph,
    boundary_before: &HashSet<HalfedgeId>,
) -> HashSet<HalfedgeId> {
    boundary_halves(mesh)
        .into_iter()
        .filter(|he_id| !boundary_before.contains(he_id))
        .collect()
}

/// The live face-less halfedges that start at `region`'s vertices.
///
/// A removed face's members all start at one of its vertices, so as long as the
/// region holds the vertices of every face a removal batch takes out, this sees
/// the whole rim that batch opens - at the cost of the region's edges rather than
/// a scan of the mesh. The test build checks it against [`boundary_halves`].
pub(crate) fn boundary_halves_around(mesh: &MeshGraph, region: &[VertexId]) -> HashSet<HalfedgeId> {
    region_halfedges(mesh, region)
        .filter_map(|(_, he_id, he)| he.is_boundary().then_some(he_id))
        .collect()
}

/// Every live outgoing halfedge of `region`'s vertices, with the vertex it leaves.
fn region_halfedges<'a>(
    mesh: &'a MeshGraph,
    region: &'a [VertexId],
) -> impl Iterator<Item = (VertexId, HalfedgeId, &'a Halfedge)> + 'a {
    region.iter().flat_map(move |&v_id| {
        mesh.outgoing_halfedges
            .get(v_id)
            .into_iter()
            .flatten()
            .filter_map(move |&he_id| Some((v_id, he_id, mesh.halfedges.get(he_id)?)))
    })
}

/// [`opened_rim`] over `region` only: `boundary_halves_around(after) \ before`,
/// with `before` taken by [`boundary_halves_around`] over the same region.
pub(crate) fn opened_rim_around(
    mesh: &MeshGraph,
    region: &[VertexId],
    boundary_before: &HashSet<HalfedgeId>,
) -> HashSet<HalfedgeId> {
    boundary_halves_around(mesh, region)
        .into_iter()
        .filter(|he_id| !boundary_before.contains(he_id))
        .collect()
}

/// The border as the merge found it, keyed by vertex pair, for its hole closer -
/// taken over the region around the merge instead of the whole mesh.
///
/// It knows every edge at a vertex of `region`. An edge with neither end in the
/// region counts as an old border ([`BoundaryAtEntry`]), so a region that turned
/// out too small could only make the closer leave a hole open, never seal a
/// border it knows nothing about. The test build checks, merge by merge, that the
/// closer sees exactly what a whole-mesh snapshot would.
pub(crate) struct RegionBoundary {
    pairs: HashSet<(VertexId, VertexId)>,
    region: HashSet<VertexId>,
    /// The whole-mesh snapshot, for that check.
    #[cfg(test)]
    pub(crate) whole_mesh: HashSet<(VertexId, VertexId)>,
}

impl RegionBoundary {
    pub(crate) fn new(mesh: &MeshGraph, region: &[VertexId]) -> Self {
        let pairs = region_halfedges(mesh, region)
            .filter(|(_, _, he)| he.is_border_edge(mesh))
            .map(|(v_id, _, he)| edge_key(v_id, he.end_vertex))
            .collect();

        Self {
            pairs,
            region: region.iter().copied().collect(),
            #[cfg(test)]
            whole_mesh: mesh.boundary_edge_pairs(),
        }
    }

    /// The snapshot carried through the vertex merges and splits the op made, like
    /// [`VertexRenames::resolve`].
    pub(crate) fn resolve(&self, renames: &VertexRenames) -> Self {
        Self {
            pairs: renames.resolve(&self.pairs),
            region: renames.resolve_region(&self.region),
            #[cfg(test)]
            whole_mesh: renames.resolve(&self.whole_mesh),
        }
    }

    /// The live region vertices, where the closer looks for opened boundary.
    pub(crate) fn region(&self) -> impl Iterator<Item = VertexId> + '_ {
        self.region.iter().copied()
    }
}

impl BoundaryAtEntry for RegionBoundary {
    fn was_boundary(&self, a: VertexId, b: VertexId) -> bool {
        self.pairs.contains(&edge_key(a, b))
            || (!self.region.contains(&a) && !self.region.contains(&b))
    }
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
    let cap = mesh.outgoing_halfedges.get(end_v_id).map_or(0, Vec::len) + 2;

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

/// The rim halves as an ordered ring: for every structure in `halves`, walks it
/// head-first (chains from their heads, then the loops) in ascending id order
/// and collects each half's end vertex and the half itself.
///
/// This is the live replacement for the pre-removal `one_ring()` arrays: every
/// vertex and halfedge in it is on the boundary the removals actually opened,
/// so the band planner can no longer index stale or off-rim elements.
#[cfg(test)]
pub(crate) fn ring_from_halves(
    mesh: &MeshGraph,
    halves: &HashSet<HalfedgeId>,
) -> (Vec<VertexId>, Vec<HalfedgeId>) {
    let mut starts: Vec<HalfedgeId> = halves.iter().copied().collect();
    starts.sort_unstable();

    let mut has_predecessor = HashSet::default();
    for &he_id in &starts {
        if let RimStep::Continue(next) = next_rim_half(mesh, he_id, halves) {
            has_predecessor.insert(next);
        }
    }

    let mut visited = HashSet::default();
    let mut v_ids = Vec::with_capacity(halves.len());
    let mut he_ids = Vec::with_capacity(halves.len());

    let mut walk = |start: HalfedgeId, visited: &mut HashSet<HalfedgeId>| {
        let mut current = start;
        loop {
            let Some(he) = mesh.halfedges.get(current) else {
                return;
            };
            he_ids.push(current);
            v_ids.push(he.end_vertex);
            match next_rim_half(mesh, current, halves) {
                RimStep::Continue(next) => {
                    if !visited.insert(next) {
                        return;
                    }
                    current = next;
                }
                RimStep::ChainEnd | RimStep::Isolated => return,
            }
        }
    };

    for &start in starts.iter().filter(|he| !has_predecessor.contains(*he)) {
        if visited.insert(start) {
            walk(start, &mut visited);
        }
    }
    for &start in &starts {
        if visited.insert(start) {
            walk(start, &mut visited);
        }
    }

    (v_ids, he_ids)
}
