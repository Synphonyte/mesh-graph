mod add;
mod cleanup;
mod collapse;
mod edit;
mod merge_one_ring;
mod query;
mod remove;
mod subdivide;
mod transform;

use std::{cmp::Reverse, collections::BinaryHeap};

pub use add::*;
use hashbrown::HashMap;
pub use merge_one_ring::*;

use ordered_float::OrderedFloat;

use crate::{HalfedgeId, MeshGraph, utils::unwrap_or_return};

/// The outcome of [`MeshGraph::collapse_until_edges_above_min_length`] and
/// [`MeshGraph::subdivide_until_edges_below_max_length`].
///
/// Both operations are bounded: they will not grind indefinitely on a mesh they
/// cannot fix. This says which way the operation ended, so a caller that needs a
/// clean mesh can react instead of guessing an iteration count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeLengthCleanup {
    /// No edge violates the length threshold any more. A caller looping until the
    /// mesh is clean can stop.
    Converged,

    /// Edges still violate the threshold and the operation could not reduce them
    /// further — either the work bound was reached, or the remaining edges cannot
    /// be changed at all (a collapse that would invert a face, for instance).
    ///
    /// Calling again may help in the first case but not the second, and the two
    /// are not distinguished here. A caller that cannot accept a dirty mesh should
    /// report a failure rather than spin.
    Stalled,
}

impl EdgeLengthCleanup {
    /// `true` only for [`Self::Converged`].
    #[inline]
    pub fn converged(self) -> bool {
        self == Self::Converged
    }
}

/// Which end of the length range [`PendingEdges::pop_live`] yields first.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum PendingOrder {
    /// Shortest edge first, for `collapse_until_edges_above_min_length`.
    ShortestFirst,
    /// Longest edge first, for `subdivide_until_edges_below_max_length`.
    LongestFirst,
}

/// The set of edges still violating a length threshold, ordered so that the
/// next edge to operate on can be taken in `O(log n)` instead of a linear scan.
///
/// Keys are canonical halfedge ids (`he_id.min(twin_id)`, one entry per
/// undirected edge, as produced by [`MeshGraph::halfedges_map`]); values are
/// squared lengths.
///
/// The `HashMap` is the authoritative state and the heap is only an index into
/// it, kept honest by two rules:
///
/// 1. [`Self::insert`] is the *only* way to add an entry, and it pushes to the
///    heap whenever it changes the map. That makes "every live entry has a
///    matching heap entry" structural rather than something each call site has
///    to remember.
/// 2. [`Self::remove`] touches only the map. The orphaned heap entry is
///    detected and discarded when it surfaces (lazy invalidation), which is far
///    cheaper than finding and erasing it eagerly.
///
/// Ties are broken towards the smallest halfedge id in both orderings, so the
/// choice is deterministic. (The linear scans this replaced used a strict
/// comparison and so resolved ties by `hashbrown` iteration order, which
/// `foldhash`'s randomly-seeded state makes vary between runs.)
pub(crate) struct PendingEdges {
    lengths: HashMap<HalfedgeId, f32>,
    /// Max-heap over the sort key. `ShortestFirst` negates the length so that
    /// the shortest edge compares greatest; squared lengths are non-negative so
    /// negation is exact and strictly order-reversing.
    ///
    /// `OrderedFloat` supplies the total order. Squared edge lengths are never
    /// NaN (a threshold comparison against NaN is false in both directions, so
    /// such an edge is never admitted to the pending set), so the NaN corner of
    /// that order is unreachable here.
    heap: BinaryHeap<(OrderedFloat<f32>, Reverse<HalfedgeId>)>,
    order: PendingOrder,
}

impl PendingEdges {
    /// Wraps a pending map from [`MeshGraph::halfedges_map`].
    pub(crate) fn new(lengths: HashMap<HalfedgeId, f32>, order: PendingOrder) -> Self {
        // Build the heap in one O(n) heapify rather than n pushes.
        let heap = BinaryHeap::from(
            lengths
                .iter()
                .map(|(&he_id, &len_sqr)| (Self::key(order, len_sqr), Reverse(he_id)))
                .collect::<Vec<_>>(),
        );

        Self {
            lengths,
            heap,
            order,
        }
    }

    #[inline]
    fn key(order: PendingOrder, len_sqr: f32) -> OrderedFloat<f32> {
        match order {
            PendingOrder::ShortestFirst => OrderedFloat(-len_sqr),
            PendingOrder::LongestFirst => OrderedFloat(len_sqr),
        }
    }

    #[inline]
    fn unkey(order: PendingOrder, key: OrderedFloat<f32>) -> f32 {
        match order {
            PendingOrder::ShortestFirst => -key.0,
            PendingOrder::LongestFirst => key.0,
        }
    }

    /// Records `he_id` as pending at `len_sqr`, pushing to the heap only when
    /// this actually changes the map.
    ///
    /// Skipping the push for an unchanged value is what keeps the heap from
    /// growing without bound: both callers re-examine every edge of the faces
    /// they touched, and most of those edges are already pending at exactly the
    /// length recorded for them. A re-insert after a [`Self::remove`] does
    /// change the map, so it is always pushed — which is the one way a single
    /// map entry can end up with two matching heap entries, see
    /// [`Self::pop_live`].
    pub(crate) fn insert(&mut self, he_id: HalfedgeId, len_sqr: f32) {
        if self.lengths.insert(he_id, len_sqr) != Some(len_sqr) {
            self.heap
                .push((Self::key(self.order, len_sqr), Reverse(he_id)));
        }
    }

    /// Restores the heap entry for an id that [`Self::pop_live`] returned but the
    /// caller chose not to act on.
    ///
    /// `pop_live` consumes the heap entry without touching the map, so an
    /// unacted-on id is left in the map with nothing in the heap pointing at it.
    /// Routing it back through [`Self::insert`] would not help: the map still
    /// holds the same length, so the change check would skip the push and the
    /// entry would be unreachable forever. Hence the unconditional push.
    ///
    /// A later `insert` or `remove` for the same id simply makes this entry
    /// stale, and `pop_live` discards it like any other stale entry.
    pub(crate) fn requeue(&mut self, he_id: HalfedgeId, len_sqr: f32) {
        self.heap
            .push((Self::key(self.order, len_sqr), Reverse(he_id)));
    }

    /// Drops `he_id` from the pending set. Its heap entry is left to be
    /// discarded by [`Self::pop_live`].
    pub(crate) fn remove(&mut self, he_id: &HalfedgeId) {
        self.lengths.remove(he_id);
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.lengths.is_empty()
    }

    pub(crate) fn len(&self) -> usize {
        self.lengths.len()
    }

    /// Removes and returns the extreme pending edge as `(he_id, len_sqr)`,
    /// discarding stale heap entries on the way.
    ///
    /// An entry is stale when the map no longer holds that id (it was removed)
    /// or holds a different length for it (a later `insert` superseded it). The
    /// comparison is exact `f32` equality, which is right because the value
    /// being compared is the very same `f32` that was stored, not a
    /// recomputation of it.
    ///
    /// The contract is "yields *a* live entry", not "yields each id once": a
    /// [`Self::remove`] followed by an `insert` of a bit-identical length leaves
    /// the pre-`remove` heap entry matching the map again, so the id can come
    /// back twice. Callers must tolerate that — both re-run their own check on
    /// the second yield, which is why it stays harmless — and must not derive an
    /// entry count from how many times `pop_live` returned (the drained-heap
    /// `debug_assert!` in `collapse_until_edges_above_min_length` allows for the
    /// surplus). Exact repeats are not exotic: collapsing to edge midpoints on a
    /// regular grid regenerates lengths like `1.0` and `0.25` bit-for-bit.
    ///
    /// Returns `None` once no live entry remains, which — given rule 1 above —
    /// means the pending set is empty. Both callers `debug_assert!` that.
    pub(crate) fn pop_live(&mut self) -> Option<(HalfedgeId, f32)> {
        while let Some((key, Reverse(he_id))) = self.heap.pop() {
            if self.lengths.get(&he_id) == Some(&Self::unkey(self.order, key)) {
                return Some((he_id, Self::unkey(self.order, key)));
            }
        }

        None
    }
}

impl MeshGraph {
    pub fn halfedges_map(&mut self, predicate: impl Fn(f32) -> bool) -> HashMap<HalfedgeId, f32> {
        let mut halfedges_map = HashMap::new();

        for (he_id, he) in &self.halfedges {
            let twin_id = unwrap_or_return!(he.twin, "Twin missing", halfedges_map);

            let id = he_id.min(twin_id);

            if halfedges_map.contains_key(&id) {
                continue;
            }
            let len_sqr = he.length_squared(self);

            if predicate(len_sqr) {
                halfedges_map.insert(id, len_sqr);
            }
        }

        halfedges_map
    }
}

#[cfg(test)]
mod pending_edges_test {
    use super::*;
    use crate::utils::build_grid;

    /// Three distinct, live canonical halfedge ids to key entries with.
    fn ids() -> Vec<HalfedgeId> {
        let g = build_grid(2);
        let mut ids: Vec<HalfedgeId> = g.halfedges.keys().take(3).collect();
        ids.sort();
        ids
    }

    #[test]
    fn pops_shortest_first() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::ShortestFirst);
        p.insert(id[0], 9.0);
        p.insert(id[1], 1.0);
        p.insert(id[2], 5.0);

        assert_eq!(p.pop_live(), Some((id[1], 1.0)));
        p.remove(&id[1]);
        assert_eq!(p.pop_live(), Some((id[2], 5.0)));
    }

    #[test]
    fn pops_longest_first() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::LongestFirst);
        p.insert(id[0], 9.0);
        p.insert(id[1], 1.0);
        p.insert(id[2], 5.0);

        assert_eq!(p.pop_live(), Some((id[0], 9.0)));
        p.remove(&id[0]);
        assert_eq!(p.pop_live(), Some((id[2], 5.0)));
    }

    #[test]
    fn removed_entries_are_skipped() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::LongestFirst);
        p.insert(id[0], 9.0);
        p.insert(id[1], 1.0);

        p.remove(&id[0]);

        assert_eq!(p.pop_live(), Some((id[1], 1.0)));
    }

    #[test]
    fn superseded_entries_are_skipped() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::LongestFirst);
        p.insert(id[0], 9.0);
        // Same id re-inserted shorter: the stale 9.0 heap entry must not be returned.
        p.insert(id[0], 2.0);

        assert_eq!(p.pop_live(), Some((id[0], 2.0)));
        p.remove(&id[0]);
        assert_eq!(p.pop_live(), None);
    }

    /// `pop_live` consumes the heap entry but leaves the map entry, so an id the
    /// caller declines to act on is unreachable until it is requeued. Routing it
    /// back through `insert` is NOT enough — the map already holds that length, so
    /// the change check would skip the push and the entry would be lost.
    #[test]
    fn requeue_makes_a_declined_entry_reachable_again() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::ShortestFirst);
        p.insert(id[0], 1.0);

        let popped = p.pop_live().expect("entry is live");
        assert_eq!(popped, (id[0], 1.0));

        // Declining to act on it: still pending in the map, but gone from the heap.
        assert!(!p.is_empty());
        assert_eq!(p.pop_live(), None);

        // `insert` cannot resurrect it, because the value is unchanged.
        p.insert(id[0], 1.0);
        assert_eq!(
            p.pop_live(),
            None,
            "insert must not resurrect a declined id"
        );

        p.requeue(id[0], 1.0);
        assert_eq!(p.pop_live(), Some((id[0], 1.0)));
    }

    /// A requeued entry that is later removed must not come back.
    #[test]
    fn requeue_then_remove_stays_gone() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::ShortestFirst);
        p.insert(id[0], 1.0);
        p.pop_live();
        p.requeue(id[0], 1.0);

        p.remove(&id[0]);

        assert_eq!(p.pop_live(), None);
        assert!(p.is_empty());
    }

    /// `remove` leaves the heap entry behind, so re-inserting a *bit-identical*
    /// length makes that orphan match the map again and the id is yielded twice.
    /// Documented behaviour rather than a bug: both callers re-run their own check
    /// on the second yield. It is pinned here because the drained-heap
    /// `debug_assert!` in `collapse_until_edges_above_min_length` has to allow for
    /// the resulting surplus - an equality there would panic on valid state.
    #[test]
    fn reinsert_after_remove_can_yield_the_same_id_twice() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::ShortestFirst);
        p.insert(id[0], 1.0);

        p.remove(&id[0]);
        // Same bit pattern, so the orphaned heap entry matches the map once more.
        p.insert(id[0], 1.0);

        assert_eq!(p.pop_live(), Some((id[0], 1.0)));
        assert_eq!(
            p.pop_live(),
            Some((id[0], 1.0)),
            "the orphaned entry is live again and must surface, not be silently dropped"
        );
        assert_eq!(p.len(), 1, "two heap entries, still one pending edge");

        p.remove(&id[0]);
        assert_eq!(p.pop_live(), None);
    }

    /// Re-inserting an unchanged value must not push, or the re-check passes would
    /// grow the heap without bound.
    #[test]
    fn unchanged_reinsert_does_not_grow_the_heap() {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), PendingOrder::LongestFirst);
        p.insert(id[0], 4.0);
        let heap_len = p.heap.len();

        for _ in 0..100 {
            p.insert(id[0], 4.0);
        }

        assert_eq!(p.heap.len(), heap_len);
        assert_eq!(p.len(), 1);
    }

    #[test]
    fn seeding_from_a_map_preserves_order() {
        let id = ids();
        let mut seed = HashMap::new();
        seed.insert(id[0], 3.0);
        seed.insert(id[1], 7.0);

        let mut p = PendingEdges::new(seed, PendingOrder::LongestFirst);
        assert_eq!(p.len(), 2);
        assert_eq!(p.pop_live(), Some((id[1], 7.0)));
    }

    /// Equal lengths must resolve to the smallest halfedge id. The linear scans
    /// this replaced used a strict comparison, so ties fell to `hashbrown`
    /// iteration order and varied between runs; every other test here uses
    /// distinct lengths, which leaves the `Reverse(HalfedgeId)` half of the sort
    /// key unexercised. Ids go in out of order so a heap that leaked insertion
    /// order would fail too.
    fn assert_ties_break_towards_smallest_id(order: PendingOrder) {
        let id = ids();
        let mut p = PendingEdges::new(HashMap::new(), order);
        p.insert(id[2], 4.0);
        p.insert(id[0], 4.0);
        p.insert(id[1], 4.0);

        for expected in &id {
            assert_eq!(
                p.pop_live(),
                Some((*expected, 4.0)),
                "{order:?} did not break the tie towards the smallest id"
            );
            p.remove(expected);
        }

        assert!(p.is_empty());
    }

    #[test]
    fn shortest_first_breaks_ties_towards_the_smallest_id() {
        assert_ties_break_towards_smallest_id(PendingOrder::ShortestFirst);
    }

    #[test]
    fn longest_first_breaks_ties_towards_the_smallest_id() {
        assert_ties_break_towards_smallest_id(PendingOrder::LongestFirst);
    }
}
