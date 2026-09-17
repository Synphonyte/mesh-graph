//! The rim-first band planner: plan the stitch band from the live rim `H`.
//!
//! The op runs this planner on the rim the removals opened, after tearing the
//! already-connected strip out (strips-first), and uses the result when
//! [`RimPlan::is_complete`] holds. When it does not, the old ring-range planner's
//! faces stand, so this planner is purely additive: it can close holes the old
//! one left, never take away a plan that worked.
//!
//! The shape of the model follows the measurements in the handoff:
//!
//! - `H` is walked head-first (`rim::next_rim_half`) in ascending id order, so
//!   the structure is deterministic and independent of slotmap hash order.
//! - Each walk is split into maximal same-side **runs** (`h1` / `h2` / `strip`).
//!   A walk whose last half ends where its first half starts closes at a pinch
//!   even when `next_rim_half` reports `ChainEnd`; the wrap runs are then merged
//!   so a run is maximal *cyclically* too. That is what turns the sharp
//!   `radius 1, +5` case (walked as `h2, h1, h2`) into the two-run bigon it
//!   really is.
//! - A cyclic structure whose runs are two opposite-side arcs is planned with
//!   the existing Bresenham machinery
//!   ([`MeshGraph::plan_connect_pair`]), so the band keeps the shape the old
//!   planner aimed for. Any other cyclic lobe is fan-triangulated from its first
//!   boundary vertex; a walk that revisits a vertex (a pinch inside one walk,
//!   e.g. the extracted weld case) is split into simple lobes at the repeated
//!   vertex first.
//! - What is deliberately *not* guessed at: open chains, whose ends run into a
//!   pre-existing border of the cut region. They are reported as `unhandled`
//!   instead of producing faces from a boundary model that does not describe
//!   them.
//!
//! Measured on the captured corpus: this planner closes the extracted weld case
//! and moved `radius 1, +5` and `radius 1, +6` out of `OPEN_WELD_CASES` into
//! `WELD_CASES`. With it driving the op, the whole corpus is green with the
//! repair nets disabled (`MESH_GRAPH_MERGE_NETS=0`), which was the Phase 2
//! oracle.
//!
//! External soak (freestyle-sculpt via a local `[patch]` to this commit):
//!
//! - shipping order, 500 iterations: 500 watertight. Before this change the
//!   same soak failed at iteration 110 with 7 boundary halfedges.
//! - reversed order (temporary `std::cmp::Reverse` on the merge-candidate sort
//!   key, then reverted), 500 iterations: 500 watertight. Before this change it
//!   produced a failing case on every run.
//! - debug build, 200 iterations: 200 watertight with every `debug_assert!`
//!   live, including the star-removal completeness assert and the planned-face
//!   transaction asserts.
//!
//! The two remaining ex-open cases (`radius 2, +4` and its mirror) cannot be
//! planned from their fixture at all, and not for want of machinery: their rim
//! is an **open chain**. A triangulation's boundary is always a union of closed
//! curves, so no set of triangles can consume an open chain using only rim
//! halves and interior edges. Closing it would mean consuming the pre-existing
//! border halfedges the chain runs into - which would seal a genuine open border
//! and is exactly what `close_triangular_holes` is written to avoid. See
//! `tests::test_open_weld_chain_cases_are_cut_truncated`.
//!
//! They are unfaithful fixtures, not defects: the extractor's
//! `NO STANDALONE REPRO` fallback prints the widest cut *without* minimising it,
//! which drops one-ring faces and leaves a *ring* vertex on the cut. They now
//! live in `CUT_TRUNCATED_CASES` and `OPEN_WELD_CASES` is empty. Re-running that
//! same reversed soak with this planner temporarily disabled and the capture
//! radius raised reproduces a genuine, minimised case at radius 1; it is
//! `WELD_CASES`'s `re-extracted reversed weld order, radius 1, +4` entry, and
//! this planner closes it (`boundary 13 -> 13`).

use hashbrown::{HashMap, HashSet};

use crate::{HalfedgeId, MeshGraph, VertexId};

use super::rim::{self, RimStep};
use super::{ConnectPair, ConnectPairCap, PlannedFace, PlannedFaceOrder};

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

    let mut has_predecessor = HashSet::new();
    for &he_id in &starts {
        if let RimStep::Continue(next) = rim::next_rim_half(mesh, he_id, halves) {
            has_predecessor.insert(next);
        }
    }

    let mut visited = HashSet::new();
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
        let cyclic = cyclic
            || match (
                ordered.first().and_then(|he_id| start_vertex(mesh, *he_id)),
                ordered.last().and_then(|he_id| end_vertex(mesh, *he_id)),
            ) {
                (Some(a), Some(b)) => a == b,
                _ => false,
            };

        structures.push(structure_from_walk(mesh, ordered, cyclic, h1, h2));
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

fn start_vertex(mesh: &MeshGraph, he_id: HalfedgeId) -> Option<VertexId> {
    mesh.halfedges.get(he_id)?.start_vertex(mesh)
}

fn end_vertex(mesh: &MeshGraph, he_id: HalfedgeId) -> Option<VertexId> {
    mesh.halfedges.get(he_id).map(|he| he.end_vertex)
}

/// Group an ordered walk into maximal same-side runs, merging the wrap runs of a
/// cyclic walk.
fn structure_from_walk(
    mesh: &MeshGraph,
    ordered: Vec<HalfedgeId>,
    cyclic: bool,
    h1: &HashSet<HalfedgeId>,
    h2: &HashSet<HalfedgeId>,
) -> RimStructure {
    let mut runs: Vec<RimRun> = Vec::new();
    for he_id in ordered.iter().copied() {
        let side = side_of(he_id, h1, h2);
        let Some(start) = start_vertex(mesh, he_id) else {
            continue;
        };
        let Some(end) = end_vertex(mesh, he_id) else {
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
        *first = RimRun {
            side: first.side,
            vertices: merged,
        };
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
        let mut directed: HashSet<(VertexId, VertexId)> = HashSet::new();
        for face in &self.faces {
            directed.insert((face.v1, face.new_he_v1));
            directed.insert((face.new_he_v1, face.new_he_v2));
            directed.insert((face.new_he_v2, face.v1));
        }

        halves
            .iter()
            .copied()
            .filter(|he_id| {
                let (Some(start), Some(end)) =
                    (start_vertex(mesh, *he_id), end_vertex(mesh, *he_id))
                else {
                    return false;
                };
                directed.contains(&(start, end))
            })
            .collect()
    }

    /// Whether this plan consumes the whole rim and nothing else.
    ///
    /// The op only replaces the old planner's faces with this plan when this
    /// returns `true`, so it is the safety contract, not a heuristic:
    ///
    /// - every structure was planned (no `unhandled`),
    /// - no face is degenerate and no directed edge appears twice (a
    ///   consistently oriented triangulation has each interior edge once per
    ///   direction and each boundary edge once),
    /// - every rim halfedge is consumed,
    /// - and no face runs along a *pre-existing* border halfedge, which would
    ///   seal a genuine open border rather than the hole the merge opened.
    pub fn is_complete(&self, mesh: &MeshGraph, halves: &HashSet<HalfedgeId>) -> bool {
        if !self.unhandled.is_empty() {
            return false;
        }
        if self.faces.is_empty() {
            return halves.is_empty();
        }

        let mut directed: HashSet<(VertexId, VertexId)> = HashSet::new();
        for face in &self.faces {
            let v = [face.v1, face.new_he_v1, face.new_he_v2];
            if v[0] == v[1] || v[1] == v[2] || v[0] == v[2] {
                return false;
            }
            for edge in [(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
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
            for he_id in mesh.halfedges_from_to(start, end) {
                if let Some(he) = mesh.halfedges.get(he_id)
                    && he.face.is_none()
                    && !halves.contains(&he_id)
                {
                    return false;
                }
            }
        }

        true
    }
}

/// Plan the band from the live rim.
pub(crate) fn plan(
    mesh: &MeshGraph,
    halves: &HashSet<HalfedgeId>,
    h1: &HashSet<HalfedgeId>,
    h2: &HashSet<HalfedgeId>,
) -> RimPlan {
    let mut plan = RimPlan::default();

    for structure in rim_structures(mesh, halves, h1, h2) {
        plan_structure(mesh, &structure, &mut plan);
    }

    plan
}

fn plan_structure(mesh: &MeshGraph, structure: &RimStructure, plan: &mut RimPlan) {
    if !structure.cyclic {
        plan.unhandled
            .push(format!("open chain of {} runs", structure.runs.len()));
        return;
    }

    if structure.runs.len() == 2
        && structure.runs[0].side != RimSide::Strip
        && structure.runs[1].side != RimSide::Strip
    {
        plan_bigon(mesh, structure, plan);
        return;
    }

    plan_fan(structure, plan);
}

/// A two-run cyclic structure: one side-1 arc and one side-2 arc between the
/// same two junctions. Reuse the old Bresenham pairing so the band geometry is
/// the same shape the old planner aimed for.
fn plan_bigon(mesh: &MeshGraph, structure: &RimStructure, plan: &mut RimPlan) {
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
    // the shared junction, exactly like the old `ConnectPairCap::Closed`.
    let mut b_reversed = b.vertices.clone();
    b_reversed.reverse();

    let range_a = ConnectPair::new(
        0..=a.vertices.len() - 1,
        a.vertices.len(),
        0..=b_reversed.len() - 1,
        b_reversed.len(),
        ConnectPairCap::Closed,
    );

    plan.faces
        .extend(mesh.plan_connect_pair(&range_a, [&a.vertices, &b_reversed]));
}

/// Fan-triangulate a closed rim lobe from its first boundary vertex.
///
/// A cyclic structure is the boundary of a disk (possibly pinched at a vertex
/// it shares with another structure). A fan consumes every boundary half and
/// adds only interior diagonals, so it cannot open boundary. It is the fallback
/// for lobes the range-pair model cannot express - a single-side loop, or a
/// boundary that crosses a detached strip rung.
///
/// A closed walk that revisits a vertex is a figure-eight patch: several lobes
/// meeting at that vertex (the extracted weld case is one). It is decomposed
/// into simple sub-cycles first and each is fanned independently; the lobes then
/// share the pinch vertex, which is the connectivity the removed patch had.
fn plan_fan(structure: &RimStructure, plan: &mut RimPlan) {
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

        let v0 = cycle[0];
        for i in 1..cycle.len() - 1 {
            // `Single` inserts each triangle independently: a fan's consecutive
            // faces share the closing edge, not the edge2/edge1 pair that
            // `add_planned_faces` chains on.
            plan.faces.push(PlannedFace::new(
                v0,
                cycle[i],
                cycle[i + 1],
                PlannedFaceOrder::Single,
            ));
        }
    }
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
    let mut first_index: HashMap<VertexId, usize> = HashMap::new();

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
