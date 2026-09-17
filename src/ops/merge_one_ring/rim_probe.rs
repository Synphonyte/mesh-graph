//! Test-only collector for the boundary-delta taxonomy of a single merge.
//!
//! The op pushes one [`RimProbe`] per band merge (thread-local, so parallel
//! tests do not interleave). Probes are measurements, not assertions: Phase 0
//! records how far the current rebuild is from "closes exactly what it opened"
//! before Phase 1/2 make that structural.

use std::cell::RefCell;

use hashbrown::HashSet;

use crate::{HalfedgeId, MeshGraph};

use super::rim;
use super::rim::RimStep;

/// `before` and `rim` are filled in by the op as the removals happen;
/// `classify` walks the rim right before the planned faces go in, `finish`
/// computes the outcome once they are.
#[derive(Debug, Clone, Default)]
pub(crate) struct RimProbe {
    /// Boundary halves immediately before the first removal.
    pub before: HashSet<HalfedgeId>,
    /// `opened_rim` deltas of the removal batches (stars + strips).
    pub rim: HashSet<HalfedgeId>,

    /// Closed rim loops (walk returns to its start).
    pub rim_loops: usize,
    /// Rim chains ending on a pre-existing border.
    pub rim_chains: usize,
    /// Rim halves whose walk is broken (isolated or non-manifold).
    pub rim_bad: usize,
    /// Details of the broken walks, for diagnostics.
    pub rim_bad_details: Vec<String>,
    /// Star-rim halves that the strip removal freed again (their edge vanished
    /// entirely), so they are not part of the live rim.
    pub rim_freed: usize,

    /// Rim halves that are still boundary after the planned faces were added.
    pub unconsumed: HashSet<HalfedgeId>,
    /// Edges whose two halves are both face-less after the planned faces.
    pub wire: Vec<HalfedgeId>,
    /// Boundary halves that are neither in `before` nor in `rim`.
    pub new_boundary: HashSet<HalfedgeId>,
}

impl RimProbe {
    /// Walk the rim with [`rim::next_rim_half`] and count loops, border-ending
    /// chains and broken starts. Call while all rim halves are live (before the
    /// planned faces are added).
    pub(crate) fn classify(&mut self, mesh: &MeshGraph) {
        // A halfedge the star removal opened can be freed again by the strip
        // removal (both halves of its edge belonged to a removed face). Those
        // are no longer part of the rim; keep them only as a count.
        let before_filter = self.rim.len();
        self.rim.retain(|he_id| mesh.halfedges.contains_key(*he_id));
        self.rim_freed = before_filter - self.rim.len();

        // Ascending seeds: a walk entered mid-chain would otherwise score a
        // legitimate chain as `rim_bad`, and which half is entered first must
        // not depend on hash order.
        let mut starts: Vec<HalfedgeId> = self.rim.iter().copied().collect();
        starts.sort_unstable();

        let mut visited = HashSet::new();
        for &start in &starts {
            if !visited.insert(start) {
                continue;
            }
            let mut current = start;
            loop {
                match rim::next_rim_half(mesh, current, &self.rim) {
                    RimStep::Continue(next) => {
                        if !visited.insert(next) {
                            if next == start {
                                self.rim_loops += 1;
                            } else {
                                self.rim_bad += 1;
                            }
                            break;
                        }
                        current = next;
                    }
                    RimStep::ChainEnd => {
                        self.rim_chains += 1;
                        break;
                    }
                    RimStep::Isolated => {
                        self.rim_bad += 1;
                        let end_v = mesh.halfedges.get(current).map(|he| he.end_vertex);
                        self.rim_bad_details
                            .push(format!("{current:?} -> {end_v:?}: isolated"));
                        break;
                    }
                }
            }
        }
    }

    pub(crate) fn finish(&mut self, mesh: &MeshGraph) {
        let boundary_now = rim::boundary_halves(mesh);

        self.unconsumed = self
            .rim
            .iter()
            .copied()
            .filter(|he_id| boundary_now.contains(he_id))
            .collect();

        self.wire = boundary_now
            .iter()
            .copied()
            // Pre-existing wire edges are not this merge's doing.
            .filter(|he_id| !self.before.contains(he_id))
            .filter(|he_id| {
                mesh.halfedges
                    .get(*he_id)
                    .and_then(|he| he.twin)
                    .and_then(|twin_id| mesh.halfedges.get(twin_id))
                    .is_some_and(|twin| twin.face.is_none())
            })
            .collect();
        self.wire.sort();

        self.new_boundary = boundary_now
            .into_iter()
            .filter(|he_id| !self.before.contains(he_id) && !self.rim.contains(he_id))
            .collect();
    }

    /// A stable, printable one-line summary.
    pub(crate) fn summary(&self) -> String {
        let count = |set: &HashSet<HalfedgeId>| set.len();
        let wire: Vec<String> = self.wire.iter().map(|he_id| format!("{he_id:?}")).collect();
        format!(
            "before {} rim {} (loops {} chains {} bad {} freed {} {:?}) -> unconsumed {} new_boundary {} wire {:?}",
            self.before.len(),
            self.rim.len(),
            self.rim_loops,
            self.rim_chains,
            self.rim_bad,
            self.rim_freed,
            self.rim_bad_details,
            count(&self.unconsumed),
            count(&self.new_boundary),
            wire,
        )
    }
}

thread_local! {
    static RECORDS: RefCell<Vec<RimProbe>> = const { RefCell::new(Vec::new()) };
}

pub(crate) fn push(probe: RimProbe) {
    RECORDS.with(|records| records.borrow_mut().push(probe));
}

pub(crate) fn drain() -> Vec<RimProbe> {
    RECORDS.with(|records| records.borrow_mut().drain(..).collect())
}
