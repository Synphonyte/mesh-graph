//! The band planner: plan the stitch band from the live rim `H`.
//!
//! The op runs this planner on the rim the removals opened, after tearing the
//! already-connected strip out, and builds the result when [`RimPlan::incompleteness`]
//! finds nothing missing. Otherwise no band is built, and the op's cleanup and
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
//!   arcs evenly. A walk that revisits a vertex (a pinch) is zipped the same way
//!   when its faces fit. When the zip repeats a directed edge, or one of its rungs
//!   runs along an edge that already joins two of its vertices outside the hole, the
//!   bigon falls back to the lobe planner, which splits a pinched walk at the repeat
//!   first. Any other closed lobe is zipped between two arcs of its boundary split at
//!   roughly opposite vertices, through the same machinery. A lobe's split moves along
//!   its boundary until none of its chords runs along an edge that already joins two of
//!   its vertices outside the hole, or that an earlier structure of the same rim
//!   plans.
//! - A lobe no split fits is triangulated by the shortest-diagonal dynamic program
//!   over its own vertices that avoids those edges ([`RimFit::triangulate`]), which
//!   finds a fitting triangulation whenever one exists. When none does, the lobe is
//!   fanned from a new vertex at its centroid ([`CenterFan`]), which has no edges
//!   that could clash.
//! - Two *disjoint* single-side loops - the two stars' rings sharing no vertex -
//!   are stitched to each other by an open-cap band seeded at the nearest
//!   position ([`plan_disjoint_band`]); when every seed's rungs clash with existing
//!   edges, [`RimFit::band`] searches all bands for the shortest that fits. Capping each loop on its own is a valid
//!   closed mesh that the completeness check cannot tell from the real band, but it
//!   is not a merge.
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
    /// Added through [`Self::push_faces`], which keeps `edges` in step.
    pub faces: Vec<PlannedFace>,
    /// Lobes no triangulation of their own vertices can fill with edges that fit - a
    /// diagonal already exists outside the hole, or another structure plans it: each is
    /// filled by a fan from a new vertex, which the op adds. Added through
    /// [`Self::push_center_fan`].
    pub center_fans: Vec<CenterFan>,
    /// Human-readable reasons a structure could not be planned.
    pub unhandled: Vec<String>,
    /// The directed edges planned so far: the faces' and the center fans' sides (the
    /// spokes run to vertices that don't exist yet, so nothing can repeat them).
    edges: HashSet<(VertexId, VertexId)>,
    /// Test-only: lobes no fitting triangulation was found for, with the diagonals that
    /// didn't fit.
    #[cfg(test)]
    pub notes: Vec<String>,
}

/// A lobe filled by a fan from a new vertex at `center` (see [`RimPlan::center_fans`]).
#[derive(Debug, Clone)]
pub(crate) struct CenterFan {
    pub cycle: Vec<VertexId>,
    /// The centroid of the cycle's positions.
    pub center: Vec3,
}

impl CenterFan {
    /// The fan around `cycle`, or `None` when a vertex of it has no position.
    fn new(mesh: &MeshGraph, cycle: &[VertexId]) -> Option<Self> {
        let mut sum = Vec3::ZERO;
        for v_id in cycle {
            sum += *mesh.positions.get(*v_id)?;
        }
        Some(Self {
            cycle: cycle.to_vec(),
            center: sum / cycle.len() as f32,
        })
    }
}

/// Why [`RimPlan::incompleteness`] refuses a plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Incomplete {
    /// A structure could not be planned (see [`RimPlan::unhandled`]).
    Unhandled,
    /// Nothing is planned for a non-empty rim.
    NoFaces,
    Degenerate,
    /// A directed edge is planned twice.
    PlannedTwice(VertexId, VertexId),
    /// A rim half no planned face consumes.
    NotConsumed(HalfedgeId),
    /// A planned edge doesn't fit ([`RimFit::edge`]).
    DoesNotFit(VertexId, VertexId),
    /// A planned edge whose reverse is neither planned nor on the rim.
    OpensBoundary(VertexId, VertexId),
}

impl RimPlan {
    /// Adds `faces` to the plan.
    pub(super) fn push_faces(&mut self, faces: impl IntoIterator<Item = PlannedFace>) {
        for face in faces {
            self.edges.extend(face.directed_edges());
            self.faces.push(face);
        }
    }

    /// Plans a fan from a new vertex.
    fn push_center_fan(&mut self, fan: CenterFan) {
        self.edges.extend(cycle_sides(&fan.cycle));
        self.center_fans.push(fan);
    }

    /// Whether `start -> end` is already planned.
    fn has_planned(&self, start: VertexId, end: VertexId) -> bool {
        self.edges.contains(&(start, end))
    }

    /// Whether `faces` can join the plan as they are: no degenerate face, every
    /// directed edge fits ([`RimFit::edge`]), none is already planned, and none
    /// appears twice within `faces` itself.
    ///
    /// The degenerate and duplicate checks are the same contract
    /// [`Self::incompleteness`] enforces, applied to the candidate so a generator
    /// that reuses an edge cannot produce a plan that is only refused at the end.
    fn fits(&self, fit: RimFit, faces: &[PlannedFace]) -> bool {
        let mut planned = HashSet::default();
        faces.iter().all(|face| {
            !face.is_degenerate()
                && face.directed_edges().into_iter().all(|(start, end)| {
                    planned.insert((start, end))
                        && fit.edge(start, end)
                        && !self.has_planned(start, end)
                })
        })
    }

    /// The faces the plan adds: its own, plus one per side of each center fan.
    #[cfg(test)]
    pub fn face_count(&self) -> usize {
        self.faces.len()
            + self
                .center_fans
                .iter()
                .map(|fan| fan.cycle.len())
                .sum::<usize>()
    }

    /// The rim halves consumed by the plan: a half `u -> v` is consumed when some
    /// planned triangle or center fan carries the directed edge `u -> v`.
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
            .chain(
                self.center_fans
                    .iter()
                    .flat_map(|fan| cycle_sides(&fan.cycle)),
            )
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

    /// Whether [`Self::incompleteness`] finds nothing missing.
    #[cfg(test)]
    pub fn is_complete(&self, mesh: &MeshGraph, halves: &HashSet<HalfedgeId>) -> bool {
        self.incompleteness(mesh, halves).is_none()
    }

    /// Why this plan doesn't consume exactly the whole rim, or `None` when it does.
    ///
    /// The op only builds a plan this returns `None` for, so it is the safety contract,
    /// not a heuristic:
    ///
    /// - every structure was planned (no `unhandled`),
    /// - no face is degenerate and no directed edge appears twice (a
    ///   consistently oriented triangulation has each interior edge once per
    ///   direction and each boundary edge once),
    /// - every rim halfedge is consumed,
    /// - every planned edge whose reverse is not planned is a rim halfedge, so the
    ///   plan opens no boundary of its own,
    /// - and every planned edge fits ([`RimFit::edge`]): none runs along a
    ///   *pre-existing* border halfedge, which would seal a genuine open border
    ///   rather than the hole the merge opened, and none duplicates a halfedge
    ///   that already carries a face, which would put a second face on it.
    ///
    /// Recounts the edges from `faces` and `center_fans` rather than trusting the
    /// incremental set, so a plan built by hand is checked the same way.
    pub fn incompleteness(
        &self,
        mesh: &MeshGraph,
        halves: &HashSet<HalfedgeId>,
    ) -> Option<Incomplete> {
        if !self.unhandled.is_empty() {
            return Some(Incomplete::Unhandled);
        }
        if self.faces.is_empty() && self.center_fans.is_empty() {
            return (!halves.is_empty()).then_some(Incomplete::NoFaces);
        }

        // A center fan consumes its lobe's sides; its spokes pair up among themselves.
        let mut directed: HashSet<(VertexId, VertexId)> = HashSet::default();
        for fan in &self.center_fans {
            for (start, end) in cycle_sides(&fan.cycle) {
                if !directed.insert((start, end)) {
                    return Some(Incomplete::PlannedTwice(start, end));
                }
            }
        }
        for face in &self.faces {
            if face.is_degenerate() {
                return Some(Incomplete::Degenerate);
            }
            for (start, end) in face.directed_edges() {
                if !directed.insert((start, end)) {
                    return Some(Incomplete::PlannedTwice(start, end));
                }
            }
        }

        for he_id in halves {
            if !mesh.halfedges.contains_key(*he_id) {
                continue;
            }
            if !mesh
                .halfedge_endpoints(*he_id)
                .is_some_and(|edge| directed.contains(&edge))
            {
                return Some(Incomplete::NotConsumed(*he_id));
            }
        }

        let fit = RimFit { mesh, halves };
        for &(start, end) in &directed {
            if !fit.edge(start, end) {
                return Some(Incomplete::DoesNotFit(start, end));
            }
            if !directed.contains(&(end, start)) && !fit.is_rim(start, end) {
                return Some(Incomplete::OpensBoundary(start, end));
            }
        }

        None
    }

    /// `reason` with what the planner noted about it, for the rim probe.
    #[cfg(test)]
    pub fn describe(&self, reason: Incomplete) -> String {
        let mut description = format!("{reason:?}");
        if !self.unhandled.is_empty() {
            description.push_str(&format!(" {:?}", self.unhandled));
        }
        for note in &self.notes {
            description.push_str(&format!("; {note}"));
        }
        description
    }
}

/// The directed sides `cycle[i] -> cycle[i + 1]` of a closed cycle.
pub(super) fn cycle_sides(cycle: &[VertexId]) -> impl Iterator<Item = (VertexId, VertexId)> + '_ {
    (0..cycle.len()).map(|i| (cycle[i], cycle[(i + 1) % cycle.len()]))
}

/// What a planned edge has to fit: the mesh as the removals left it, and the rim they
/// opened.
#[derive(Clone, Copy)]
struct RimFit<'a> {
    mesh: &'a MeshGraph,
    halves: &'a HashSet<HalfedgeId>,
}

impl RimFit<'_> {
    /// The live halfedges `start -> end`.
    fn halfedges(&self, start: VertexId, end: VertexId) -> impl Iterator<Item = HalfedgeId> + '_ {
        self.mesh
            .outgoing_halfedges
            .get(start)
            .into_iter()
            .flatten()
            .copied()
            .filter(move |he_id| {
                self.mesh
                    .halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.end_vertex == end)
            })
    }

    /// Whether a planned face may carry the directed edge `start -> end`: every halfedge
    /// the mesh already has in that direction must be a face-less rim half, the one the
    /// face consumes. A faced one would get a second face on the same edge, and a
    /// face-less one off the rim is a pre-existing border the merge must not seal.
    fn edge(&self, start: VertexId, end: VertexId) -> bool {
        self.halfedges(start, end)
            .all(|he_id| self.halves.contains(&he_id) && self.mesh.halfedges[he_id].face.is_none())
    }

    /// Whether a new edge between `u` and `v` fits in both directions.
    fn new_edge(&self, u: VertexId, v: VertexId) -> bool {
        self.edge(u, v) && self.edge(v, u)
    }

    /// Whether `start -> end` is a rim half.
    fn is_rim(&self, start: VertexId, end: VertexId) -> bool {
        self.halfedges(start, end)
            .any(|he_id| self.halves.contains(&he_id))
    }

    /// The band between the loops `arc1` and `arc2` (oriented as in [`plan_disjoint_band`])
    /// with the shortest total rung length whose rungs all fit, or `None` when there is
    /// none.
    ///
    /// A band is a closed walk over the rungs `(arc1[i], arc2[j])` in which each face
    /// advances along one of the loops, once around both: a staircase from `(0, 0)` to
    /// `(m, n)` on the grid, read modulo `(m, n)`. The staircase lands on a rung twice
    /// exactly when it runs a whole row or column - one vertex fanned over the entire
    /// other loop and back onto its first rung - so those are excluded.
    ///
    /// Every band has a rung at `arc1[0]` that the walk leaves along `arc1` (the last of
    /// `arc1[0]`'s fan), so the walk starts there: the first `k` steps go along `arc1`
    /// (`1 <= k < m`, or `arc2[t]` would take the whole of `arc1`), and the walk may then
    /// reach the last column only past column `k`, or it would reuse the rungs of that
    /// first run. The row of `arc1[0]` is never revisited, since the walk leaves it at
    /// once. Dynamic programming over the rest of the grid, once per `(t, k)`.
    fn band(&self, arc1: &[VertexId], arc2: &[VertexId]) -> Option<Vec<PlannedFace>> {
        let (m, n) = (arc1.len(), arc2.len());
        let position = |v_id: VertexId| self.mesh.positions.get(v_id).copied();
        let mut rung_length = vec![None; m * n];
        for (a, &u) in arc1.iter().enumerate() {
            for (b, &v) in arc2.iter().enumerate() {
                if self.new_edge(u, v) {
                    rung_length[a * n + b] = Some(position(u)?.distance(position(v)?));
                }
            }
        }

        // cost[i * width + j]: the shortest walk from the starting rung to rung (i, j),
        // counting each rung once - the starting rung at the end, where the walk returns
        // to it as (m, n).
        let width = n + 1;
        let mut cost = vec![f32::INFINITY; (m + 1) * width];
        let mut best: Option<(f32, Vec<PlannedFace>)> = None;
        for t in 0..n {
            if rung_length[t].is_none() {
                continue;
            }
            let rung = |i: usize, j: usize| rung_length[(i % m) * n + (t + j) % n];

            for k in 1..m {
                cost.fill(f32::INFINITY);
                cost[0] = 0.0;
                for i in 1..=k {
                    let Some(length) = rung(i, 0) else {
                        break;
                    };
                    cost[i * width] = cost[(i - 1) * width] + length;
                }
                if !cost[k * width].is_finite() {
                    // A longer first run passes the same missing rung.
                    break;
                }

                for i in k..=m {
                    for j in 1..=n {
                        if j == n && i <= k {
                            continue;
                        }
                        let Some(length) = rung(i, j) else {
                            continue;
                        };
                        let along_second = cost[i * width + j - 1];
                        let along_first = if i > k {
                            cost[(i - 1) * width + j]
                        } else {
                            f32::INFINITY
                        };
                        cost[i * width + j] = along_first.min(along_second) + length;
                    }
                }

                let total = cost[m * width + n];
                if !total.is_finite() || best.as_ref().is_some_and(|(b, _)| *b <= total) {
                    continue;
                }

                // Walk back from (m, n), one face per step.
                let a = |i: usize| arc1[i % m];
                let b = |j: usize| arc2[(t + j) % n];
                let mut faces = Vec::with_capacity(m + n);
                let (mut i, mut j) = (m, n);
                while (i, j) != (0, 0) {
                    let along_first =
                        j == 0 || (i > k && cost[(i - 1) * width + j] <= cost[i * width + j - 1]);
                    if along_first {
                        faces.push(PlannedFace::new(
                            a(i - 1),
                            a(i),
                            b(j),
                            PlannedFaceOrder::Single,
                        ));
                        i -= 1;
                    } else {
                        faces.push(PlannedFace::new(
                            a(i),
                            b(j),
                            b(j - 1),
                            PlannedFaceOrder::Single,
                        ));
                        j -= 1;
                    }
                }
                faces.reverse();
                best = Some((total, faces));
            }
        }

        best.map(|(_, faces)| faces)
    }

    /// The shortest-diagonal triangulation of the simple cycle `cycle` that only adds edges
    /// that fit and that `plan` doesn't have yet, or `None` when there is none.
    ///
    /// Dynamic programming over the sub-polygons `cycle[i..=j]`; each triangle `(i, k, j)`
    /// keeps the cycle's winding, so the cycle's own sides are consumed in their direction.
    fn triangulate(&self, cycle: &[VertexId], plan: &RimPlan) -> Option<Vec<PlannedFace>> {
        let n = cycle.len();
        if n < 3 {
            return None;
        }
        let positions: Vec<Vec3> = cycle
            .iter()
            .map(|v_id| self.mesh.positions.get(*v_id).copied())
            .collect::<Option<_>>()?;

        // The sides, cycle[i] -> cycle[i + 1], are rim halves; one an earlier structure
        // already consumed can't be consumed again.
        if cycle_sides(cycle).any(|(start, end)| plan.has_planned(start, end)) {
            return None;
        }

        // A diagonal is a new edge in both directions.
        let diagonal_fits = |a: usize, b: usize| {
            let (u, v) = (cycle[a], cycle[b]);
            self.new_edge(u, v) && !plan.has_planned(u, v) && !plan.has_planned(v, u)
        };
        let length = |a: usize, b: usize| positions[a].distance(positions[b]);

        // cost[i][j]: the shortest total diagonal length triangulating cycle[i..=j], whose
        // closing edge (i, j) is a side or a fitting diagonal.
        let mut cost = vec![vec![f32::INFINITY; n]; n];
        let mut split = vec![vec![0; n]; n];
        for i in 0..n - 1 {
            cost[i][i + 1] = 0.0;
        }
        for span in 2..n {
            for i in 0..n - span {
                let j = i + span;
                if span < n - 1 && !diagonal_fits(i, j) {
                    continue;
                }
                for k in i + 1..j {
                    let total = cost[i][k] + cost[k][j] + length(i, k) + length(k, j);
                    if total < cost[i][j] {
                        cost[i][j] = total;
                        split[i][j] = k;
                    }
                }
            }
        }
        if !cost[0][n - 1].is_finite() {
            return None;
        }

        let mut faces = Vec::with_capacity(n - 2);
        let mut stack = vec![(0, n - 1)];
        while let Some((i, j)) = stack.pop() {
            if j - i < 2 {
                continue;
            }
            let k = split[i][j];
            faces.push(PlannedFace::new(
                cycle[i],
                cycle[k],
                cycle[j],
                PlannedFaceOrder::Single,
            ));
            stack.push((k, j));
            stack.push((i, k));
        }
        Some(faces)
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
    let fit = RimFit { mesh, halves };
    let structures = rim_structures(mesh, halves, h1, h2);

    // The two merged stars' rings need not touch: when the rings share no
    // vertex, the rim is one closed loop per star, and the two must be stitched
    // to each *other*. Fanning each loop on its own would cap the two stars
    // separately - a valid closed mesh with no boundary, which the completeness
    // check cannot tell from the real band, but not a merge.
    if let Some((first, second)) = disjoint_single_side_pair(&structures) {
        plan_disjoint_band(fit, first, second, &mut plan);
        return plan;
    }

    for structure in structures {
        plan_structure(fit, &structure, &mut plan);
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
///
/// A rung is a new edge between the rings, but the two rings can already be joined by an
/// edge outside the hole; a rung along it would put a second pair of faces on it. So the
/// seed moves away from the nearest vertex, alternating sides, until every face fits
/// ([`RimPlan::fits`]), and failing that [`RimFit::band`] searches every band. When none
/// fits, the nearest seed is kept and [`RimPlan::incompleteness`] refuses the plan.
fn plan_disjoint_band(fit: RimFit, first: &RimRun, second: &RimRun, plan: &mut RimPlan) {
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
    let Some(target) = fit.mesh.positions.get(arc1[0]).copied() else {
        plan.unhandled.push("missing position".to_string());
        return;
    };
    let start2 = nearest_index(
        target,
        ring2
            .iter()
            .enumerate()
            .filter_map(|(idx, v_id)| fit.mesh.positions.get(*v_id).map(|pos| (idx, *pos))),
    );

    // The two loops bound one annulus (a closed surface minus the two stars), so
    // they run in opposite directions when the region is kept on the left:
    // `arc1` is the walk direction, `arc2` walks the other ring backwards from
    // the seed.
    let arc2_from = |seed: usize| -> Vec<VertexId> {
        (0..wrap2)
            .map(|step| ring2[(seed + wrap2 - step) % wrap2])
            .collect()
    };
    let band = |seed: usize| {
        ConnectPair::whole(arc1.len(), wrap2, ConnectPairCap::Open)
            .plan_faces([arc1, &arc2_from(seed)])
    };

    // Seeds by distance from the nearest one along the ring: 0, +1, -1, +2, ...
    let seeds = (0..wrap2).map(|step| {
        let shift = step.div_ceil(2);
        if step % 2 == 1 {
            (start2 + shift) % wrap2
        } else {
            (start2 + wrap2 - shift) % wrap2
        }
    });

    let mut nearest = None;
    for seed in seeds {
        let faces = band(seed);
        if plan.fits(fit, &faces) {
            plan.push_faces(faces);
            return;
        }
        nearest.get_or_insert(faces);
    }

    let faces = fit
        .band(arc1, &arc2_from(0))
        .or(nearest)
        .unwrap_or_default();
    plan.push_faces(faces);
}

fn plan_structure(fit: RimFit, structure: &RimStructure, plan: &mut RimPlan) {
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
        // joins its two vertices outside the hole, or the zip repeats a directed edge
        // (a pinched walk), zip it as a lobe instead, which can move its chords.
        let mut bigon = RimPlan::default();
        plan_bigon(structure, &mut bigon);
        if !bigon.unhandled.is_empty() || plan.fits(fit, &bigon.faces) {
            plan.push_faces(bigon.faces);
            plan.unhandled.extend(bigon.unhandled);
            return;
        }
    }

    plan_lobe(fit, structure, plan);
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

    plan.push_faces(range_a.plan_faces([&a.vertices, &b_reversed]));
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
fn plan_lobe(fit: RimFit, structure: &RimStructure, plan: &mut RimPlan) {
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
        plan_simple_lobe(fit, &cycle, plan);
    }
}

/// Zip one simple closed boundary cycle into triangles with two opposite arcs.
///
/// The zip's chords are new edges, but two vertices of the lobe can already be joined
/// outside the hole (a thin region, or a second strip between the rings), and an
/// earlier structure of the same rim can already plan an edge between them. A chord
/// along either would put a second pair of faces on it. The split is therefore tried at
/// each head in walk order, and the first whose faces all fit ([`RimPlan::fits`]) is
/// taken. When none does, the lobe is triangulated by [`RimFit::triangulate`], which
/// finds a fitting triangulation whenever one exists, and failing that it is fanned from
/// a new center vertex. Only when that fails too is the first split kept, and
/// [`RimPlan::incompleteness`] refuses the plan.
fn plan_simple_lobe(fit: RimFit, cycle: &[VertexId], plan: &mut RimPlan) {
    let mut first = None;
    let mut rotated = cycle.to_vec();

    for _ in 0..cycle.len() {
        let faces = zip_lobe(&rotated);
        if plan.fits(fit, &faces) {
            plan.push_faces(faces);
            return;
        }
        first.get_or_insert(faces);
        rotated.rotate_left(1);
    }

    if let Some(faces) = fit.triangulate(cycle, plan) {
        plan.push_faces(faces);
        return;
    }

    // No triangulation of the lobe's own vertices fits: every one needs a diagonal that
    // already exists outside the hole or that another structure plans. A new vertex has
    // no edges yet, so a fan from it fits as long as no other structure consumes one of
    // the lobe's sides. Without positions there is no place to put it: the lobe stays
    // unplanned, and the plan is refused.
    if !cycle_sides(cycle).any(|(start, end)| plan.has_planned(start, end)) {
        match CenterFan::new(fit.mesh, cycle) {
            Some(fan) => plan.push_center_fan(fan),
            None => plan
                .unhandled
                .push("lobe vertex without a position".to_string()),
        }
        return;
    }

    #[cfg(test)]
    {
        let n = cycle.len();
        let mut blocked = Vec::new();
        for a in 0..n {
            for b in a + 2..n {
                if a == 0 && b == n - 1 {
                    continue;
                }
                let (u, v) = (cycle[a], cycle[b]);
                let why = if plan.has_planned(u, v) || plan.has_planned(v, u) {
                    "planned"
                } else if !fit.new_edge(u, v) {
                    "exists"
                } else {
                    continue;
                };
                blocked.push(format!("{a}-{b} {why}"));
            }
        }
        plan.notes.push(format!(
            "lobe of {n} {cycle:?}: no fitting triangulation, blocked [{}]",
            blocked.join(", ")
        ));
    }

    plan.push_faces(first.unwrap_or_default());
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

#[cfg(test)]
mod tests {
    use glam::vec3;

    use super::*;

    /// The rim of a lone cycle: one face-less halfedge per side, nothing else.
    fn cycle_rim(mesh: &mut MeshGraph, cycle: &[VertexId]) -> HashSet<HalfedgeId> {
        cycle_sides(cycle)
            .map(|(start, end)| mesh.add_edge(start, end).unwrap().start_to_end_he_id)
            .collect()
    }

    fn assert_no_edge_twice(faces: &[PlannedFace]) {
        let mut directed = HashSet::default();
        for edge in faces.iter().flat_map(PlannedFace::directed_edges) {
            assert!(
                directed.insert(edge),
                "directed edge {edge:?} planned twice"
            );
        }
    }

    /// Two loops of three with no edge between them, so every band fits. One vertex of the
    /// second loop sits right next to the first loop and the other two far away: the
    /// cheapest staircase fans that one vertex over the whole first loop and back onto its
    /// first rung, which is not a band. A valid one exists and must be found.
    #[test]
    fn test_band_never_reuses_a_rung() {
        let mut mesh = MeshGraph::new();
        let arc1 = [
            mesh.add_vertex(vec3(0.0, 0.0, 0.0)),
            mesh.add_vertex(vec3(1.0, 0.0, 0.0)),
            mesh.add_vertex(vec3(0.5, 1.0, 0.0)),
        ];
        let arc2 = [
            mesh.add_vertex(vec3(0.5, 0.4, 0.1)),
            mesh.add_vertex(vec3(10.0, 0.0, 10.0)),
            mesh.add_vertex(vec3(10.5, 1.0, 10.0)),
        ];
        let halves = HashSet::default();
        let fit = RimFit {
            mesh: &mesh,
            halves: &halves,
        };

        let faces = fit.band(&arc1, &arc2).expect("a valid band exists");
        assert_eq!(faces.len(), 6);
        assert_no_edge_twice(&faces);
    }

    /// A hexagon whose only free diagonals are the fan from `v0`. Every zig-zag split
    /// needs one of the others, so the lobe has to fall back to a triangulation that
    /// isn't one - the fallback is strictly more general than rotating the split.
    #[test]
    fn test_lobe_falls_back_to_a_triangulation_no_split_gives() {
        let mut mesh = MeshGraph::new();
        let cycle: Vec<VertexId> = (0..6)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::TAU / 6.0;
                mesh.add_vertex(vec3(angle.cos(), angle.sin(), 0.0))
            })
            .collect();
        let halves = cycle_rim(&mut mesh, &cycle);
        for (a, b) in [(1, 3), (1, 4), (1, 5), (2, 4), (2, 5), (3, 5)] {
            mesh.add_edge(cycle[a], cycle[b]).unwrap();
        }
        let fit = RimFit {
            mesh: &mesh,
            halves: &halves,
        };

        let empty = RimPlan::default();
        let mut rotated = cycle.clone();
        for _ in 0..cycle.len() {
            assert!(
                !empty.fits(fit, &zip_lobe(&rotated)),
                "precondition: a zig-zag split fits"
            );
            rotated.rotate_left(1);
        }

        let mut plan = RimPlan::default();
        plan_simple_lobe(fit, &cycle, &mut plan);

        assert_eq!(plan.faces.len(), 4);
        assert!(plan.center_fans.is_empty());
        assert_eq!(plan.incompleteness(&mesh, &halves), None);
    }

    /// A quad whose two diagonals both exist outside it: no triangulation of its own
    /// vertices fits, so it is fanned from a new vertex at its centroid.
    #[test]
    fn test_lobe_without_a_fitting_triangulation_gets_a_center_fan() {
        let mut mesh = MeshGraph::new();
        let cycle: Vec<VertexId> = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
            .into_iter()
            .map(|(x, y)| mesh.add_vertex(vec3(x, y, 0.0)))
            .collect();
        let halves = cycle_rim(&mut mesh, &cycle);
        mesh.add_edge(cycle[0], cycle[2]).unwrap();
        mesh.add_edge(cycle[1], cycle[3]).unwrap();
        let fit = RimFit {
            mesh: &mesh,
            halves: &halves,
        };

        let mut plan = RimPlan::default();
        plan_simple_lobe(fit, &cycle, &mut plan);

        assert!(plan.faces.is_empty());
        assert_eq!(plan.center_fans.len(), 1);
        assert_eq!(plan.center_fans[0].cycle, cycle);
        assert_eq!(plan.center_fans[0].center, vec3(1.0, 1.0, 0.0));
        assert_eq!(plan.face_count(), 4);
        assert_eq!(plan.consumed_halves(&mesh, &halves), halves);
        assert_eq!(plan.incompleteness(&mesh, &halves), None);

        // Without a position there is nowhere to put the center: refused, not fanned
        // from the origin.
        mesh.positions.remove(cycle[2]);
        let fit = RimFit {
            mesh: &mesh,
            halves: &halves,
        };
        let mut plan = RimPlan::default();
        plan_simple_lobe(fit, &cycle, &mut plan);
        assert!(plan.center_fans.is_empty());
        assert_eq!(
            plan.incompleteness(&mesh, &halves),
            Some(Incomplete::Unhandled)
        );
    }
}
