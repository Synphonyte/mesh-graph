//! The band planner: plan the stitch band from the live rim `H`.
//!
//! The op runs this planner on the rim the removals opened, after tearing the
//! already-connected strip out, and builds the result when [`RimPlan::is_complete`]
//! holds. When it does not, no band is built, and the op's cleanup and
//! remaining-unconnected merge repair the region.
//!
//! The rim is used rather than the pre-removal one-rings because the rings cannot
//! describe the hole the removals actually opened: a pinched star opens more rim
//! than `one_ring()` sees, and a chain can end on the cut.
//!
//! - `H` is walked head-first (`rim::next_rim_half`) in ascending id order, so
//!   the structure is deterministic and independent of slotmap hash order.
//! - Each walk is split into maximal same-side **runs** (`h1` / `h2` / `strip`).
//!   A walk whose last half ends where its first half starts closes at a pinch
//!   even when `next_rim_half` reports `ChainEnd`; the wrap runs are then merged
//!   so a run is maximal *cyclically* too. A walk that reads `h2, h1, h2` is then
//!   the two-run bigon it really is.
//! - A cyclic structure whose runs are two opposite-side arcs (a bigon) is planned
//!   with the Bresenham machinery ([`ConnectPair::plan_faces`]), pairing the two
//!   arcs evenly. Any other closed lobe is
//!   zipped between two arcs of its boundary split at roughly opposite vertices,
//!   through the same machinery; a walk that revisits a vertex (a pinch inside one
//!   walk) is split into simple lobes at the repeated vertex first. A lobe's split
//!   moves along its boundary until none of its chords runs along an edge that
//!   already joins two of its vertices outside the hole, and a bigon whose fixed
//!   rungs do falls back to that lobe zip.
//! - Two *disjoint* single-side loops - the two stars' rings sharing no vertex -
//!   are stitched to each other by an open-cap band seeded at the nearest
//!   position ([`plan_disjoint_band`]). Capping each loop on its own is a valid
//!   closed mesh that `is_complete` cannot tell from the real band, but it is not
//!   a merge.
//! - Open chains, whose ends run into a pre-existing border of the cut region,
//!   are reported as `unhandled`, and nothing is planned for the rim. A
//!   triangulation's boundary is always a union of closed curves, so no set of
//!   triangles can consume an open chain using only rim halves and interior
//!   edges; closing it would mean consuming the pre-existing border halfedges the
//!   chain runs into, which would seal a genuine open border (see
//!   `tests::test_open_weld_chain_cases_are_cut_truncated`).

use crate::hash::{HashMap, HashSet};
use glam::Vec3;

use crate::{HalfedgeId, MeshGraph, VertexId};

use super::rim::{self, RimStep};
use super::{ConnectPair, ConnectPairCap, PlannedFace};

/// Which ring a rim half bounds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RimSide {
    /// Opened by removing the first merged vertex's star.
    First,
    /// Opened by removing the second merged vertex's star.
    Second,
    /// Opened by removing an already-connected strip: a rung joining a side-1
    /// and a side-2 vertex.
    Strip,
}

/// A maximal same-side stretch of a rim walk.
#[derive(Debug, Clone)]
pub(crate) struct RimRun {
    pub side: RimSide,
    /// The start vertex of the first half, then the end vertex of every half.
    pub vertices: Vec<VertexId>,
}

/// One connected rim walk (a loop, a chain, or a chain that closes at a pinch).
#[derive(Debug, Clone)]
pub(crate) struct RimStructure {
    /// The walk closes back onto its start vertex, so its runs are maximal on a
    /// cycle (the first and last run merge when they share a side).
    pub cyclic: bool,
    pub runs: Vec<RimRun>,
}

fn side_of(he_id: HalfedgeId, h1: &HashSet<HalfedgeId>, h2: &HashSet<HalfedgeId>) -> RimSide {
    if h1.contains(&he_id) {
        RimSide::First
    } else if h2.contains(&he_id) {
        RimSide::Second
    } else {
        RimSide::Strip
    }
}

/// Walk `halves` into structures with maximal same-side runs.
///
/// Deterministic: starts are visited in ascending id order, chains from their
/// heads first (a half with no predecessor), then whatever is left (loops).
pub(crate) fn rim_structures(
    mesh: &MeshGraph,
    halves: &HashSet<HalfedgeId>,
    h1: &HashSet<HalfedgeId>,
    h2: &HashSet<HalfedgeId>,
) -> Vec<RimStructure> {
    let mut starts: Vec<HalfedgeId> = halves.iter().copied().collect();
    starts.sort_unstable();

    let mut has_predecessor = HashSet::default();
    for &he_id in &starts {
        if let RimStep::Continue(next) = rim::next_rim_half(mesh, he_id, halves) {
            has_predecessor.insert(next);
        }
    }

    let mut visited = HashSet::default();
    let mut structures = Vec::new();

    let mut walk = |start: HalfedgeId, visited: &mut HashSet<HalfedgeId>| {
        let mut ordered: Vec<HalfedgeId> = Vec::new();
        let mut current = start;
        let cyclic;
        loop {
            ordered.push(current);
            match rim::next_rim_half(mesh, current, halves) {
                RimStep::Continue(next) => {
                    if !visited.insert(next) {
                        cyclic = next == start;
                        break;
                    }
                    current = next;
                }
                RimStep::ChainEnd | RimStep::Isolated => {
                    cyclic = false;
                    break;
                }
            }
        }

        // A chain whose ends meet at a pinch vertex closes geometrically even
        // though the boundary walk stops there (the continuation is a
        // pre-existing border half).
        let first_start = ordered
            .first()
            .and_then(|he_id| mesh.halfedge_endpoints(*he_id))
            .map(|(start, _)| start);
        let last_end = ordered
            .last()
            .and_then(|he_id| mesh.halfedges.get(*he_id))
            .map(|he| he.end_vertex);
        let cyclic = cyclic || (first_start.is_some() && first_start == last_end);

        structures.push(structure_from_walk(mesh, &ordered, cyclic, h1, h2));
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

    structures
}

/// Group an ordered walk into maximal same-side runs, merging the wrap runs of a
/// cyclic walk.
fn structure_from_walk(
    mesh: &MeshGraph,
    ordered: &[HalfedgeId],
    cyclic: bool,
    h1: &HashSet<HalfedgeId>,
    h2: &HashSet<HalfedgeId>,
) -> RimStructure {
    let mut runs: Vec<RimRun> = Vec::new();
    for &he_id in ordered {
        let side = side_of(he_id, h1, h2);
        let Some((start, end)) = mesh.halfedge_endpoints(he_id) else {
            continue;
        };

        match runs.last_mut() {
            Some(run) if run.side == side => {
                run.vertices.push(end);
            }
            _ => runs.push(RimRun {
                side,
                vertices: vec![start, end],
            }),
        }
    }

    if cyclic && runs.len() > 1 && runs.first().map(|r| r.side) == runs.last().map(|r| r.side) {
        // Merge the wrap: last -> first, sharing the junction vertex.
        let last = runs.pop().expect("runs.len() > 1");
        let first = runs.first_mut().expect("runs.len() > 1");
        let mut merged = last.vertices;
        merged.extend(first.vertices.iter().skip(1).copied());
        first.vertices = merged;
    }

    RimStructure { cyclic, runs }
}

/// What the rim-first planner produced for one rim.
#[derive(Debug, Default)]
pub(crate) struct RimPlan {
    pub faces: Vec<PlannedFace>,
    /// Human-readable reasons a structure could not be planned.
    pub unhandled: Vec<String>,
}

impl RimPlan {
    /// Number of rim halves consumed by the planned faces: a half `u -> v` is
    /// consumed when some planned triangle carries the directed edge `u -> v`.
    #[cfg(test)]
    pub fn consumed_halves(
        &self,
        mesh: &MeshGraph,
        halves: &HashSet<HalfedgeId>,
    ) -> HashSet<HalfedgeId> {
        let directed: HashSet<(VertexId, VertexId)> = self
            .faces
            .iter()
            .flat_map(PlannedFace::directed_edges)
            .collect();

        halves
            .iter()
            .copied()
            .filter(|he_id| {
                mesh.halfedge_endpoints(*he_id)
                    .is_some_and(|edge| directed.contains(&edge))
            })
            .collect()
    }

    /// Whether this plan consumes the whole rim and nothing else.
    ///
    /// The op only builds this plan when this returns `true`, so it is the safety
    /// contract, not a heuristic:
    ///
    /// - every structure was planned (no `unhandled`),
    /// - no face is degenerate and no directed edge appears twice (a
    ///   consistently oriented triangulation has each interior edge once per
    ///   direction and each boundary edge once),
    /// - every rim halfedge is consumed,
    /// - every planned edge whose reverse is not planned is a rim halfedge, so the
    ///   plan opens no boundary of its own,
    /// - and every planned edge fits ([`edge_fits`]): none runs along a
    ///   *pre-existing* border halfedge, which would seal a genuine open border
    ///   rather than the hole the merge opened, and none duplicates a halfedge
    ///   that already carries a face, which would put a second face on it.
    pub fn is_complete(&self, mesh: &MeshGraph, halves: &HashSet<HalfedgeId>) -> bool {
        if !self.unhandled.is_empty() {
            return false;
        }
        if self.faces.is_empty() {
            return halves.is_empty();
        }

        let mut directed: HashSet<(VertexId, VertexId)> = HashSet::default();
        for face in &self.faces {
            if face.is_degenerate() {
                return false;
            }
            for edge in face.directed_edges() {
                if !directed.insert(edge) {
                    return false;
                }
            }
        }

        for he_id in halves {
            let Some(he) = mesh.halfedges.get(*he_id) else {
                continue;
            };
            let Some(start) = he.start_vertex(mesh) else {
                return false;
            };
            if !directed.contains(&(start, he.end_vertex)) {
                return false;
            }
        }

        for &(start, end) in &directed {
            if !edge_fits(mesh, start, end, halves) {
                return false;
            }

            if !directed.contains(&(end, start))
                && !mesh
                    .halfedges_from_to(start, end)
                    .iter()
                    .any(|he_id| halves.contains(he_id))
            {
                return false;
            }
        }

        true
    }
}

/// Whether a planned face may carry the directed edge `start -> end`: every halfedge the
/// mesh already has in that direction must be a face-less rim half, the one the face
/// consumes. A faced one would get a second face on the same edge, and a face-less one
/// off the rim is a pre-existing border the merge must not seal.
fn edge_fits(
    mesh: &MeshGraph,
    start: VertexId,
    end: VertexId,
    halves: &HashSet<HalfedgeId>,
) -> bool {
    mesh.halfedges_from_to(start, end).into_iter().all(|he_id| {
        mesh.halfedges
            .get(he_id)
            .is_none_or(|he| he.face.is_none() && halves.contains(&he_id))
    })
}

/// Whether every edge of `faces` fits ([`edge_fits`]).
fn faces_fit(mesh: &MeshGraph, faces: &[PlannedFace], halves: &HashSet<HalfedgeId>) -> bool {
    faces.iter().all(|face| {
        face.directed_edges()
            .into_iter()
            .all(|(start, end)| edge_fits(mesh, start, end, halves))
    })
}

/// Plan the band from the live rim.
pub(crate) fn plan(
    mesh: &MeshGraph,
    halves: &HashSet<HalfedgeId>,
    h1: &HashSet<HalfedgeId>,
    h2: &HashSet<HalfedgeId>,
) -> RimPlan {
    let mut plan = RimPlan::default();
    let structures = rim_structures(mesh, halves, h1, h2);

    // The two merged stars' rings need not touch: when the rings share no
    // vertex, the rim is one closed loop per star, and the two must be stitched
    // to each *other*. Fanning each loop on its own would cap the two stars
    // separately - a valid closed mesh with no boundary, which `is_complete`
    // cannot tell from the real band, but not a merge.
    if let Some((first, second)) = disjoint_single_side_pair(&structures) {
        plan_disjoint_band(mesh, first, second, &mut plan);
        return plan;
    }

    for structure in structures {
        plan_structure(mesh, &structure, halves, &mut plan);
    }

    plan
}

/// The two structures, if the rim is exactly two single-side loops with no
/// vertex in common - one per merged star.
fn disjoint_single_side_pair(structures: &[RimStructure]) -> Option<(&RimRun, &RimRun)> {
    let [first, second] = structures else {
        return None;
    };
    if !first.cyclic || !second.cyclic {
        return None;
    }
    let ([first], [second]) = (&first.runs[..], &second.runs[..]) else {
        return None;
    };

    if first.side == RimSide::Strip || second.side == RimSide::Strip {
        return None;
    }
    if first.side == second.side {
        return None;
    }

    let first_v_ids: HashSet<VertexId> = first.vertices.iter().copied().collect();
    if second
        .vertices
        .iter()
        .any(|v_id| first_v_ids.contains(v_id))
    {
        return None;
    }

    Some((first, second))
}

/// Stitch two disjoint rings with an open-cap walk: seed the
/// correspondence by nearest position from the first ring's head, then run the
/// Bresenham pairing around both loops and let the `Open` cap close the band.
fn plan_disjoint_band(mesh: &MeshGraph, first: &RimRun, second: &RimRun, plan: &mut RimPlan) {
    // `ConnectPair` reads inclusive index ranges that wrap with `% len`, so the
    // arcs are the rings once around, without the walk's closing duplicate.
    let wrap1 = first.vertices.len().saturating_sub(1);
    let wrap2 = second.vertices.len().saturating_sub(1);
    if wrap1 < 3 || wrap2 < 3 {
        plan.unhandled.push("degenerate ring".to_string());
        return;
    }
    let arc1 = &first.vertices[..wrap1];
    let ring2 = &second.vertices[..wrap2];

    // Nearest position, lowest index on a tie, so the seed is deterministic.
    let Some(target) = mesh.positions.get(arc1[0]).copied() else {
        plan.unhandled.push("missing position".to_string());
        return;
    };
    let start2 = nearest_index(
        target,
        ring2
            .iter()
            .enumerate()
            .filter_map(|(idx, v_id)| mesh.positions.get(*v_id).map(|pos| (idx, *pos))),
    );

    // The two loops bound one annulus (a closed surface minus the two stars), so
    // they run in opposite directions when the region is kept on the left:
    // `arc1` is the walk direction, `arc2` walks the other ring backwards from
    // the nearest vertex.
    let arc2: Vec<VertexId> = (0..wrap2)
        .map(|step| ring2[(start2 + wrap2 - step) % wrap2])
        .collect();

    let range = ConnectPair::whole(arc1.len(), arc2.len(), ConnectPairCap::Open);

    plan.faces.extend(range.plan_faces([arc1, &arc2]));
}

fn plan_structure(
    mesh: &MeshGraph,
    structure: &RimStructure,
    halves: &HashSet<HalfedgeId>,
    plan: &mut RimPlan,
) {
    if !structure.cyclic {
        plan.unhandled
            .push(format!("open chain of {} runs", structure.runs.len()));
        return;
    }

    if structure.runs.len() == 2
        && structure.runs[0].side != RimSide::Strip
        && structure.runs[1].side != RimSide::Strip
    {
        // The bigon's rungs are fixed by its two junctions. When one of them already
        // joins its two vertices outside the hole, zip it as a lobe instead, which can
        // move its chords.
        let mut bigon = RimPlan::default();
        plan_bigon(structure, &mut bigon);
        if !bigon.unhandled.is_empty() || faces_fit(mesh, &bigon.faces, halves) {
            plan.faces.extend(bigon.faces);
            plan.unhandled.extend(bigon.unhandled);
            return;
        }
    }

    plan_lobe(mesh, structure, halves, plan);
}

/// A two-run cyclic structure: one side-1 arc and one side-2 arc between the
/// same two junctions, zipped with the Bresenham pairing.
fn plan_bigon(structure: &RimStructure, plan: &mut RimPlan) {
    let a = &structure.runs[0];
    let b = &structure.runs[1];

    let (Some(a_start), Some(a_end)) = (a.vertices.first(), a.vertices.last()) else {
        plan.unhandled.push("empty arc".to_string());
        return;
    };
    let (Some(b_start), Some(b_end)) = (b.vertices.first(), b.vertices.last()) else {
        plan.unhandled.push("empty arc".to_string());
        return;
    };

    if a_end != b_start || b_end != a_start {
        plan.unhandled.push(format!(
            "bigon arcs do not close ({a_start:?}..{a_end:?} vs {b_start:?}..{b_end:?})"
        ));
        return;
    }

    if a.vertices.len() < 2 || b.vertices.len() < 2 {
        plan.unhandled.push("degenerate arc".to_string());
        return;
    }

    // Orient both arcs to start at `a.vertices[0]` so the pairing is closed at
    // the shared junction, like `ConnectPairCap::Closed`.
    let b_reversed: Vec<VertexId> = b.vertices.iter().rev().copied().collect();

    let range_a = ConnectPair::whole(a.vertices.len(), b_reversed.len(), ConnectPairCap::Closed);

    plan.faces
        .extend(range_a.plan_faces([&a.vertices, &b_reversed]));
}

/// Triangulate a closed rim lobe with the same arc-pairing machinery the band
/// uses, instead of a fan.
///
/// A cyclic structure is the boundary of a disk (possibly pinched at a vertex
/// it shares with another structure). A closed walk that revisits a vertex is a
/// figure-eight patch - several lobes meeting at that vertex (the extracted weld
/// case is one) - so it is decomposed into simple sub-cycles first; the lobes
/// then share the pinch vertex, which is the connectivity the removed patch had.
///
/// Each simple cycle is split at two roughly opposite vertices into two arcs
/// that share both endpoints, and the arcs are zipped together with the closed
/// Bresenham pairing. That consumes every boundary half and adds only interior
/// edges, and spreads the triangles over the lobe far more evenly than a fan
/// from one vertex.
fn plan_lobe(
    mesh: &MeshGraph,
    structure: &RimStructure,
    halves: &HashSet<HalfedgeId>,
    plan: &mut RimPlan,
) {
    let seq = match closed_boundary_vertices(structure) {
        Ok(seq) => seq,
        Err(reason) => {
            plan.unhandled.push(format!(
                "cyclic structure of {} runs is not a closed area: {reason}",
                structure.runs.len()
            ));
            return;
        }
    };

    for cycle in decompose_pinch_cycles(&seq) {
        if cycle.len() < 3 {
            // A lobe bounded by fewer than three vertices has no area; leaving
            // it unconsumed makes the plan incomplete, which callers treat as a
            // refusal rather than silently dropping boundary.
            continue;
        }
        plan_simple_lobe(mesh, &cycle, halves, plan);
    }
}

/// Zip one simple closed boundary cycle into triangles with two opposite arcs.
///
/// The zip's chords are new edges, but two vertices of the lobe can already be joined
/// outside the hole (a thin region, or a second strip between the rings). A chord along
/// such an edge would put a second pair of faces on it. The split is therefore tried at
/// each head in walk order, and the first whose faces all fit ([`faces_fit`]) is taken;
/// when none does, the first is kept and [`RimPlan::is_complete`] refuses the plan.
fn plan_simple_lobe(
    mesh: &MeshGraph,
    cycle: &[VertexId],
    halves: &HashSet<HalfedgeId>,
    plan: &mut RimPlan,
) {
    let mut first = None;
    let mut rotated = cycle.to_vec();

    for _ in 0..cycle.len() {
        let faces = zip_lobe(&rotated);
        if faces_fit(mesh, &faces, halves) {
            plan.faces.extend(faces);
            return;
        }
        first.get_or_insert(faces);
        rotated.rotate_left(1);
    }

    plan.faces.extend(first.unwrap_or_default());
}

/// The faces of [`plan_simple_lobe`] for the split headed at `cycle[0]`.
fn zip_lobe(cycle: &[VertexId]) -> Vec<PlannedFace> {
    let mid = cycle.len() / 2;

    // `arc_a` runs forwards from the head to `mid`; `arc_b` runs backwards from
    // the head to `mid` along the other side of the cycle.
    let arc_a = &cycle[..=mid];
    let mut arc_b: Vec<VertexId> = cycle[mid..].to_vec();
    arc_b.push(cycle[0]);
    arc_b.reverse();

    if arc_a.len() < 2 || arc_b.len() < 2 {
        return Vec::new();
    }

    let range = ConnectPair::whole(arc_a.len(), arc_b.len(), ConnectPairCap::Closed);

    range.plan_faces([arc_a, &arc_b])
}

/// The closed boundary vertex cycle of a cyclic structure, with any repeated
/// vertices kept. Errors when the walk is too small to bound an area.
fn closed_boundary_vertices(structure: &RimStructure) -> Result<Vec<VertexId>, String> {
    let mut seq: Vec<VertexId> = Vec::new();
    for (i, run) in structure.runs.iter().enumerate() {
        let start = usize::from(i > 0);
        seq.extend_from_slice(&run.vertices[start..]);
    }

    // The walk closes back onto its start vertex.
    if seq.len() >= 2 && seq.first() == seq.last() {
        seq.pop();
    }

    if seq.len() < 3 {
        return Err(format!("only {} boundary vertices", seq.len()));
    }

    Ok(seq)
}

/// Split a closed walk into simple cycles at its repeated vertices.
///
/// The first repeated vertex `v` (positions `j < i`) splits the walk into the
/// lobe `seq[j..i]` and its complement, both closed at `v`; each is split again
/// recursively. Deterministic: the scan is in walk order and only lookups touch
/// the map.
fn decompose_pinch_cycles(seq: &[VertexId]) -> Vec<Vec<VertexId>> {
    let mut first_index: HashMap<VertexId, usize> = HashMap::default();

    for (i, v_id) in seq.iter().enumerate() {
        if let Some(&j) = first_index.get(v_id) {
            let inner = &seq[j..i];
            let mut outer = seq[..j].to_vec();
            outer.extend_from_slice(&seq[i..]);

            let mut cycles = Vec::new();
            if inner.len() >= 3 {
                cycles.extend(decompose_pinch_cycles(inner));
            }
            if outer.len() >= 3 {
                cycles.extend(decompose_pinch_cycles(&outer));
            }
            return cycles;
        }
        first_index.insert(*v_id, i);
    }

    vec![seq.to_vec()]
}

/// The index of the position nearest to `target`, the lowest on a tie, or `0` when there
/// is none.
fn nearest_index(target: Vec3, positions: impl IntoIterator<Item = (usize, Vec3)>) -> usize {
    let mut nearest = 0;
    let mut min_dist_sqr = f32::INFINITY;
    for (idx, pos) in positions {
        let dist_sqr = pos.distance_squared(target);
        if dist_sqr < min_dist_sqr {
            min_dist_sqr = dist_sqr;
            nearest = idx;
        }
    }
    nearest
}
