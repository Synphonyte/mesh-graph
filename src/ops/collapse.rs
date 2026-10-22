use crate::hash::{HashMap, HashSet};
use glam::Vec3;
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    Face, FaceId, HalfedgeId, MeshGraph, ScopedCleanup, Selection, VertexId, error_none,
    ops::{EdgeLengthCleanup, PendingEdges, PendingOrder},
    utils::unwrap_or_return,
};

/// The two vertices of an edge in a fixed order, so the pair can key a set.
pub(crate) fn edge_key(a: VertexId, b: VertexId) -> (VertexId, VertexId) {
    if a < b { (a, b) } else { (b, a) }
}

/// The vertex renames an op makes, so a vertex-pair snapshot taken at its start can still be
/// compared against the mesh at its end.
///
/// The "opened" tests of the hole closers compare edges by vertex pair against such a
/// snapshot. An op that merges two vertices, or splits one, gives the old edges at those
/// vertices pairs the snapshot has never seen: they look newly opened and can be sealed. The
/// renames are recorded as they happen - cheaply, touching only the vertices involved - and
/// applied to the snapshot once, when a closer needs it ([`Self::resolve`]).
#[derive(Default)]
pub(crate) struct VertexRenames {
    /// Live vertex -> the snapshot vertices it now stands for. Only vertices a rename touched
    /// have an entry; every other vertex stands for itself.
    origins: HashMap<VertexId, Vec<VertexId>>,
}

impl VertexRenames {
    /// The snapshot vertices `v_id` stands for.
    fn origins_of(&self, v_id: VertexId) -> Vec<VertexId> {
        self.origins
            .get(&v_id)
            .cloned()
            .unwrap_or_else(|| vec![v_id])
    }

    /// `absorbed` was merged into `survivor`: its edges now run from the survivor.
    pub(crate) fn absorb(&mut self, absorbed: VertexId, survivor: VertexId) {
        if absorbed == survivor {
            return;
        }

        let carried = self.origins_of(absorbed);
        self.origins.remove(&absorbed);

        let entry = self
            .origins
            .entry(survivor)
            .or_insert_with(|| vec![survivor]);
        for v_id in carried {
            if !entry.contains(&v_id) {
                entry.push(v_id);
            }
        }
    }

    /// `new` was split off `original`: some of the original's edges now run from `new`, and
    /// which ones is not recorded, so `new` stands for everything `original` stands for.
    pub(crate) fn split(&mut self, original: VertexId, new: VertexId) {
        let carried = self.origins_of(original);
        self.origins
            .entry(original)
            .or_insert_with(|| carried.clone());

        let entry = self.origins.entry(new).or_default();
        for v_id in carried {
            if !entry.contains(&v_id) {
                entry.push(v_id);
            }
        }
    }

    /// `snapshot` with every pair carried through the recorded renames: each end replaced by
    /// all the live vertices that stand for it. A pair whose two ends became one vertex is
    /// the merged edge itself and drops.
    ///
    /// A split makes this over-inclusive (both halves of a split vertex inherit all its old
    /// edges), which can only make a closer leave a hole open, never seal an old border.
    pub(crate) fn resolve(
        &self,
        snapshot: &HashSet<(VertexId, VertexId)>,
    ) -> HashSet<(VertexId, VertexId)> {
        if self.origins.is_empty() {
            return snapshot.clone();
        }

        // Snapshot vertex -> the live vertices that stand for it.
        let mut current: HashMap<VertexId, Vec<VertexId>> = HashMap::default();
        for (live, origins) in &self.origins {
            for origin in origins {
                current.entry(*origin).or_default().push(*live);
            }
        }
        let live_of = |v_id: VertexId| -> Vec<VertexId> {
            current.get(&v_id).cloned().unwrap_or_else(|| vec![v_id])
        };

        let mut resolved = HashSet::default();
        for &(a, b) in snapshot {
            let live_b = live_of(b);
            for live_a in live_of(a) {
                for &live_b in &live_b {
                    if live_a != live_b {
                        resolved.insert(edge_key(live_a, live_b));
                    }
                }
            }
        }

        resolved
    }

    /// The vertices that stand *only* for vertices of `region`: a snapshot taken over
    /// `region` knows every edge at them.
    ///
    /// Conservative on purpose. A survivor that absorbed a region vertex but was outside
    /// the region itself also carries edges the snapshot never saw, so it is left out; an
    /// edge at it then counts as covered only through its other end.
    pub(crate) fn resolve_region(&self, region: &HashSet<VertexId>) -> HashSet<VertexId> {
        let inside = |origins: &[VertexId]| origins.iter().all(|v_id| region.contains(v_id));

        let mut resolved: HashSet<VertexId> = region
            .iter()
            .copied()
            .filter(|v_id| self.origins.get(v_id).is_none_or(|origins| inside(origins)))
            .collect();
        for (live, origins) in &self.origins {
            if inside(origins) {
                resolved.insert(*live);
            }
        }

        resolved
    }
}

/// Which edges were already a border when an op started, for the hole closers: a closer
/// only ever closes boundary that is not one of these, so it never seals a border the op
/// found open.
pub(crate) trait BoundaryAtEntry {
    /// Whether the edge between `a` and `b` was a border at entry, or is one the snapshot
    /// does not cover - which a closer must leave alone just the same.
    fn was_boundary(&self, a: VertexId, b: VertexId) -> bool;
}

/// A whole-mesh snapshot, keyed by [`edge_key`]: it covers every edge.
impl BoundaryAtEntry for HashSet<(VertexId, VertexId)> {
    fn was_boundary(&self, a: VertexId, b: VertexId) -> bool {
        self.contains(&edge_key(a, b))
    }
}

/// What one run of [`MeshGraph::close_collapse_boundary_loops`] did.
#[derive(Default)]
struct CollapseCloserPass {
    /// A face was added or a lone triangle removed.
    progressed: bool,
    /// An added face has an edge below the collapse threshold.
    left_short_edge: bool,
    /// Halfedges freed by lone-triangle removals.
    removed_halfedges: Vec<HalfedgeId>,
}

impl MeshGraph {
    /// Collapses edges until all edges have a length above the minimum length.
    ///
    /// Returns whether every edge ended up above the threshold, or some remained —
    /// either because the work bound was reached or because the survivors cannot be
    /// collapsed without inverting a face. See [`EdgeLengthCleanup`].
    ///
    /// This will schedule necessary updates to the BVH but you have to call
    /// `refit_bvh()` after the operation.
    #[instrument(skip(self))]
    pub fn collapse_until_edges_above_min_length(
        &mut self,
        min_length_squared: f32,
        marked_vertices: &mut HashSet<VertexId>,
    ) -> EdgeLengthCleanup {
        self.collapse_until_edges_above_min_length_inner(min_length_squared, None, marked_vertices)
    }

    /// Collapses the edges covered by `selection` until they are all above the minimum
    /// length. Only edges in scope are ever collapsed.
    ///
    /// The scope is [`Selection::resolve_to_halfedges`]: explicitly selected halfedges,
    /// every halfedge of a selected face, and every outgoing halfedge of a selected
    /// vertex. Collapsing an edge on the rim of the selection moves a vertex that
    /// unselected faces also use, so those faces change shape even though none of their
    /// own edges were collapsed. The topology just outside the rim can change too: the
    /// collapse's cleanup splits a vertex whose fan became non-manifold, which re-links
    /// faces on both sides of the rim, and a hole the collapse opens next to the rim is
    /// closed with new faces (or loses a lone triangle). The alternative would be a
    /// broken mesh. Elements it creates out there are added to `selection` like any
    /// other; unselected elements it destroys there are not reported, as the selection
    /// never held them.
    ///
    /// `selection` is updated in place: elements the operation destroyed are dropped and
    /// elements it created are added, so the selection keeps describing the same region
    /// and never holds a dead id.
    ///
    /// Returns what changed; see [`ScopedCleanup`].
    ///
    /// This will schedule necessary updates to the BVH but you have to call
    /// `refit_bvh()` after the operation.
    #[instrument(skip(self, selection))]
    pub fn collapse_selected_until_edges_above_min_length(
        &mut self,
        selection: &mut Selection,
        min_length_squared: f32,
        marked_vertices: &mut HashSet<VertexId>,
    ) -> ScopedCleanup {
        self.run_scoped(selection, |mg, scope| {
            mg.collapse_until_edges_above_min_length_inner(
                min_length_squared,
                Some(scope),
                marked_vertices,
            )
        })
    }

    fn collapse_until_edges_above_min_length_inner(
        &mut self,
        min_length_squared: f32,
        scope: Option<&mut HashSet<HalfedgeId>>,
        marked_vertices: &mut HashSet<VertexId>,
    ) -> EdgeLengthCleanup {
        #[cfg(feature = "instrumentation")]
        crate::set_current_op("collapse");
        #[cfg(feature = "instrumentation")]
        crate::probe_chain_begin(self);
        let pending = match scope.as_deref() {
            Some(scope) => self.halfedges_map_in(scope, |len_sqr| len_sqr < min_length_squared),
            None => self.halfedges_map(|len_sqr| len_sqr < min_length_squared),
        };
        let mut halfedges_to_collapse = PendingEdges::new(pending, PendingOrder::ShortestFirst);

        // The boundary as the op found it, keyed by *edge* rather than by halfedge id:
        // the collapse and its cleanup rebuild halfedges, so an id recorded here can
        // name an edge that still exists under a new id. Only loops this op opened may
        // be closed - an edge that was already boundary belongs to a border that is
        // meant to be open (or, in a captured replay, to the cut's rim) - so this is
        // what keeps the closer from sealing one. Filled on the first collapse: with
        // nothing to collapse the mesh cannot change, and the scan would be waste.
        let mut boundary_before: Option<HashSet<(VertexId, VertexId)>> = None;
        // The wire edges as the op found them. A wire can also *form* on an edge that was
        // already boundary on one side (the collapse takes the face off the other), and
        // such an edge is not one the op "opened", so the boundary set above does not
        // describe it: the wire has to be compared against its own snapshot.
        let mut wire_before: HashSet<(VertexId, VertexId)> = HashSet::default();
        // The vertex merges and splits the collapses make, applied to both snapshots once
        // the loop is done (see `VertexRenames`).
        let mut renames = VertexRenames::default();

        // Edges popped as shortest-pending but rejected by `can_collapse_edge_inner`.
        // They stay pending, because a later collapse can make them collapsible, but
        // re-testing them every iteration is almost pure waste: measured over the
        // production scans, 18466 rejections produced 16 eventual collapses (0.09%).
        //
        // So each entry records the tick it was rejected at, and only goes back into the
        // queue once the geometry its verdict depends on has actually moved. The entry is
        // `(edge, squared length, tick when rejected)`.
        let mut deferred: Vec<(HalfedgeId, f32, u32)> = Vec::new();

        // `can_collapse_edge_inner` rejects an edge when collapsing it would invert a
        // face, which it decides from the one-rings of the edge's two endpoints. A
        // collapse moves exactly one vertex, so a rejected edge `(A, B)` can only become
        // collapsible if the moved vertex is `A`, `B`, or a neighbour of either -
        // equivalently, if `A` or `B` lies in the moved vertex's closed one-ring.
        // Recording when each vertex last moved therefore says exactly which rejected
        // edges are worth another look.
        let mut vertex_moved_at: HashMap<VertexId, u32> = HashMap::default();
        let mut tick: u32 = 0;

        // Bound the work by the initial problem size, not the mesh size: a degenerate
        // region (e.g. a cluster of zero-length edges after a bad merge) can keep
        // re-feeding the set, which made this loop burn up to the whole halfedge count
        // while growing the mesh. Healthy runs drain at ~1.2x the initial set size.
        let budget = halfedges_to_collapse.len() * 2 + 100;

        for _ in 0..budget {
            if halfedges_to_collapse.is_empty() {
                break;
            }

            // Hand back the parked candidates whose endpoints have moved since they were
            // rejected. The rest stay parked, costing two integer lookups instead of a
            // pair of one-ring walks each.
            for (he_id, len, rejected_at) in std::mem::take(&mut deferred) {
                let Some(he) = self.halfedges.get(he_id) else {
                    // The edge is gone, so it is not pending any more either.
                    halfedges_to_collapse.remove(&he_id);
                    continue;
                };

                // If the endpoints cannot be resolved the edge is broken; retry it so the
                // usual rejection path reports it rather than parking it forever.
                let moved_since = match he.start_vertex(self) {
                    Some(start) => {
                        let moved = |v| vertex_moved_at.get(&v).copied().unwrap_or(0) > rejected_at;
                        moved(start) || moved(he.end_vertex)
                    }
                    None => true,
                };

                if moved_since {
                    halfedges_to_collapse.requeue(he_id, len);
                } else {
                    deferred.push((he_id, len, rejected_at));
                }
            }

            // Take the shortest pending edge that can actually be collapsed. Rejected
            // candidates are held in `deferred` so they are neither lost nor retried
            // within this iteration, which reproduces the previous linear scan's
            // "minimum among collapsible edges" choice.
            let mut found = None;

            while let Some((he_id, len)) = halfedges_to_collapse.pop_live() {
                // Note: this mutates the mesh even when it returns `None` - it reseeds
                // `outgoing_halfedge` on both endpoints to anchor the one-ring walk in
                // `check_inverted_faces`. Each call seeds its own endpoints before its
                // own walk, so the verdict does not depend on which candidates ran
                // before it.
                if let Some((twin_id, start_v_id, end_v_id, center)) =
                    self.can_collapse_edge_inner(he_id)
                {
                    found = Some((he_id, twin_id, start_v_id, end_v_id, center));
                    break;
                }

                deferred.push((he_id, len, tick));
            }

            let Some((min_he_id, min_twin_id, min_start_v_id, min_end_v_id, min_center)) = found
            else {
                // Couldn't find a valid halfedge to collapse. Everything still pending
                // was rejected this iteration, so no further progress is possible.
                //
                // `deferred` may hold *more* entries than the map holds keys, because
                // `pop_live` can yield the same id twice (see its docs). That surplus is
                // benign. A shortfall is not: it means a pending entry had no heap entry
                // pointing at it, i.e. a push site is missing and that edge is lost for
                // the rest of the run. Only the latter is asserted.
                debug_assert!(
                    deferred.len() >= halfedges_to_collapse.len(),
                    "heap drained with {} pending and only {} deferred - a push site is missing",
                    halfedges_to_collapse.len(),
                    deferred.len()
                );
                break;
            };

            tick += 1;

            let start_vertex_id = unwrap_or_return!(
                // checked in `can_collapse_edge_inner`
                self.halfedges[min_he_id].start_vertex(self),
                "Start vertex not found",
                EdgeLengthCleanup::Stalled
            );

            if boundary_before.is_none() {
                boundary_before = Some(self.boundary_edge_pairs());
                wire_before = self.wire_edge_pairs();
            }

            let collapse_edge_result = self.collapse_edge_inner(
                min_he_id,
                min_twin_id,
                min_start_v_id,
                min_end_v_id,
                min_center,
            );

            // Carry the snapshots through the renames this collapse made, or the old border
            // edges at the renamed vertices look newly opened. The end vertex is absorbed
            // into the start vertex; the neighborhood cleanup then runs on the start vertex
            // only, so every vertex it splits off descends from it.
            if collapse_edge_result
                .removed_vertices
                .contains(&min_end_v_id)
                && !collapse_edge_result
                    .removed_vertices
                    .contains(&min_start_v_id)
            {
                renames.absorb(min_end_v_id, min_start_v_id);
            }
            for &added_v_id in &collapse_edge_result.added_vertices {
                renames.split(min_start_v_id, added_v_id);
            }

            let vertex_neighborhoods_to_check = if collapse_edge_result.added_vertices.is_empty() {
                if collapse_edge_result.removed_halfedges.is_empty() {
                    vec![]
                } else {
                    vec![start_vertex_id]
                }
            } else {
                marked_vertices.extend(collapse_edge_result.added_vertices.iter().copied());

                let mut neighborhood = collapse_edge_result.added_vertices;
                neighborhood.push(start_vertex_id);

                neighborhood
            };

            halfedges_to_collapse.remove(&min_he_id);

            for removed_he_id in collapse_edge_result.removed_halfedges {
                halfedges_to_collapse.remove(&removed_he_id);
            }

            let mut halfedges_to_check = HashSet::default();

            for vertex_id in vertex_neighborhoods_to_check {
                let Some(outgoing_halfedges) = self.outgoing_halfedges.get(vertex_id) else {
                    // the vertex might have been removed by a previous cleanup
                    continue;
                };

                // This vertex moved, or was created, by the collapse.
                vertex_moved_at.insert(vertex_id, tick);

                for &halfedge_id in outgoing_halfedges {
                    let Some(halfedge) = self.halfedges.get(halfedge_id) else {
                        error!("Halfedge not found");
                        continue;
                    };

                    // ... and so did its one-ring, as far as the inversion guard's
                    // verdict on edges incident to those neighbours is concerned.
                    vertex_moved_at.insert(halfedge.end_vertex, tick);

                    let twin_id = unwrap_or_return!(
                        halfedge.twin,
                        "Twin not found",
                        EdgeLengthCleanup::Stalled
                    );

                    // The one-ring of a collapsed vertex reaches past the selection at
                    // its rim. Only edges in scope may be queued, or the operation would
                    // eat outward into unselected geometry. The BVH refresh below is
                    // deliberately *not* skipped for them: an out-of-scope face sharing
                    // the moved vertex has changed shape, so its stored AABB is stale
                    // whether or not its edges are eligible for collapse.
                    let in_scope = scope
                        .as_deref()
                        .is_none_or(|s| s.contains(&halfedge_id) || s.contains(&twin_id));

                    if in_scope {
                        halfedges_to_check.insert(halfedge_id.min(twin_id));
                    }

                    if let Some(face_id) = halfedge.face {
                        if let Some(face) = self.faces.get(face_id) {
                            self.bvh
                                .insert_or_update_partially(face.aabb(self), face.index, 0.0);
                        } else {
                            error!("Face not found. BVH will not be updated.");
                        }
                    }
                }
            }

            for he_id in halfedges_to_check {
                let Some(he) = self.halfedges.get(he_id) else {
                    // The pair `min` inserted this id: either the halfedge itself or
                    // its twin was dead when the pair was collected (the twin is not
                    // liveness-checked at the `min` site). Skip and report once so the
                    // collision of stale SlotMap keys can be diagnosed instead of
                    // aborting the run.
                    #[cfg(feature = "instrumentation")]
                    crate::report_dead_halfedge_in_collapse_check(self, he_id);
                    continue;
                };

                let len_sqr = he.length_squared(self);

                if len_sqr < min_length_squared {
                    halfedges_to_collapse.insert(he_id, len_sqr);
                } else {
                    halfedges_to_collapse.remove(&he_id);
                }
            }
        }

        self.rebuild_outgoing_halfedges();

        // The collapse can leave a closed region one or two faces short of
        // watertight: a twin-side face removal + the neighborhood cleanup consume
        // the star of the collapsed edge, and when the two sheets meet (coincident
        // contact vertices) the rim they leave is a small boundary loop. The merge phase
        // closes the holes it opens with `close_merge_opened_holes`; the collapse phase
        // needs the same net, or what it removes it does not put back ("the op closes exactly what it opened").
        //
        // The search runs over the boundary *this op opened* - the current boundary
        // minus the boundary it found - and every loop it closes consists of such edges
        // only. That is the exact guard, and it is why the closer needs no notion of a
        // neighborhood: a cleanup can open a hole around a vertex no recorded one-ring
        // reaches, and scoping the search to what the op moved would leave that hole open
        // (`MESH_GRAPH_COLLAPSE_DEBUG` showed exactly that shape: chains of opened
        // boundary whose endpoints were never touched).
        let mut closed_over_a_short_edge = false;

        if let Some(boundary_before) = &boundary_before {
            let boundary_before = &renames.resolve(boundary_before);
            let wire_before = &renames.resolve(&wire_before);

            // A collapse can also leave a *wire* edge: both of its halves face-less, where
            // a twin-side face removal and a neighbouring one met on the same edge. It
            // carries no surface, and while it is there the boundary walk cannot pass the
            // vertices it touches - a vertex of a wire has an extra outgoing boundary
            // halfedge, which ends the walk. Dropping it is part of closing, not a separate
            // cleanup, so the two alternate.
            loop {
                let candidates = self.opened_boundary_vertices(boundary_before);

                // Wires this op made - including the ones on edges that were already
                // boundary on one side, which `candidates` does not reach.
                let mut wire_candidates = candidates.clone();
                let mut in_wire_candidates: HashSet<VertexId> =
                    wire_candidates.iter().copied().collect();
                let mut new_wires: Vec<(VertexId, VertexId)> = self
                    .wire_edge_pairs()
                    .into_iter()
                    .filter(|pair| !wire_before.contains(pair))
                    .collect();
                // Out of a hash set; the removal order decides which slots are freed first.
                new_wires.sort_unstable();
                for (a, b) in new_wires {
                    for v_id in [a, b] {
                        if in_wire_candidates.insert(v_id) {
                            wire_candidates.push(v_id);
                        }
                    }
                }

                let removed_wires = self.remove_wire_edges(&wire_candidates);

                for he_id in &removed_wires {
                    halfedges_to_collapse.remove(he_id);
                }

                let pass = self.close_collapse_boundary_loops(
                    &candidates,
                    boundary_before,
                    min_length_squared,
                );
                closed_over_a_short_edge |= pass.left_short_edge;

                for he_id in &pass.removed_halfedges {
                    halfedges_to_collapse.remove(he_id);
                }

                // Every step shrinks the boundary (a dropped wire, a closing face, a removed
                // lone triangle), so this terminates: it stops as soon as a pass changed
                // nothing.
                if removed_wires.is_empty() && !pass.progressed {
                    break;
                }
            }
        }

        // Collapsing absorbs one endpoint into the other, so ids the caller marked can
        // name vertices that no longer exist. The set is extended with the vertices
        // cleanup creates, so it has to be pruned of the ones it destroys as well, or
        // callers are handed dead keys.
        marked_vertices.retain(|v_id| self.vertices.contains_key(*v_id));

        #[cfg(feature = "instrumentation")]
        if self.probe_chain_integrity("collapse_until_edges_above_min_length") {
            crate::state_history_push(self, "collapse_until_edges_above_min_length");
        }

        #[cfg(feature = "rerun")]
        self.log_rerun();

        // The loop only exits early once the pending set drains, so anything left in it
        // is an edge below the threshold that could not be collapsed.
        // An edge the closer added can sit below the threshold - the diagonal across a
        // contact hole is short by construction, its endpoints being the vertices the
        // collapse just brought together - and the op may not call the mesh clean then.
        if halfedges_to_collapse.is_empty() && !closed_over_a_short_edge {
            EdgeLengthCleanup::Converged
        } else {
            EdgeLengthCleanup::Stalled
        }
    }

    /// Closes the boundary loops the collapse just opened.
    ///
    /// When the sheets meet at coincident vertices, the collapse's twin-side
    /// face removal plus the neighborhood cleanup can consume the whole star of
    /// the collapsed edge and leave a boundary loop - three or four halfedges
    /// typically, six or seven for the weld's small-gap openings - so the region
    /// is some faces short of watertight. The merge phase closes its own holes the
    /// same way (`close_merge_opened_holes`, same walker).
    ///
    /// Any loop length is closed, but only loops made entirely of edges this op
    /// opened: the walk follows edges absent from `boundary_before` (keyed by vertex
    /// pair, so an edge re-created under a new halfedge id still counts as old), a
    /// vertex with no opened outgoing boundary ends it, and one with several is a
    /// pinch where the lowest id is taken. A border that is meant to be open is
    /// therefore never sealed by accident. Each face comes from
    /// [`Self::close_loop_with_fan`], which skips cuts that would fold a non-convex loop.
    ///
    /// `candidates` is [`Self::opened_boundary_vertices`] of `boundary_before`: the walk
    /// starts from those, and a hole the op opened always touches one of them.
    ///
    /// This is part of the op rather than a repair net, so `MESH_GRAPH_MERGE_NETS=0`
    /// (the merge's nets-off oracle) does not disable it: what it closes is boundary this
    /// op itself opened, not a leftover of another algorithm's removals.
    ///
    /// A three-halfedge loop around a lone triangle is not closed but emptied: the triangle
    /// is removed (see [`Self::lone_face_behind`]).
    ///
    /// The result says whether anything changed, whether a face it added left an edge below
    /// the length threshold (a diagonal across a contact hole is short by construction, and
    /// the caller reports that as a survivor rather than as a clean mesh), and which
    /// halfedges a lone-triangle removal freed.
    fn close_collapse_boundary_loops(
        &mut self,
        candidates: &[VertexId],
        boundary_before: &HashSet<(VertexId, VertexId)>,
        min_length_squared: f32,
    ) -> CollapseCloserPass {
        let mut pass = CollapseCloserPass::default();
        let mut closed = 0usize;

        // Closing a loop can shorten a longer one: fanning a quad adds the face on
        // one side of a diagonal and leaves the triangle on the other side, which
        // the next sweep picks up. Every sweep that closes something strictly
        // shrinks the boundary, so this terminates.
        loop {
            let mut progressed = false;

            for &start_v in candidates {
                if !self.vertices.contains_key(start_v) {
                    continue;
                }

                // Every halfedge of the loop it returns is one this op opened (see
                // `boundary_loop_from`), so a border that was already there can only be
                // touched by the loop, never closed over.
                let Some(loop_hes) = self.boundary_loop_from(start_v, boundary_before) else {
                    continue;
                };

                if let Some(face_id) = self.lone_face_behind(&loop_hes) {
                    let (_, removed_halfedges) = self.remove_face(face_id);
                    pass.removed_halfedges.extend(removed_halfedges);
                    pass.progressed = true;
                    progressed = true;
                    continue;
                }

                // Close the loop as a fan: one face over two consecutive halfedges, which
                // cuts the diagonal the rest of the loop folds around. Three halfedges close
                // outright; more leave the remainder for the next sweep - and the corner
                // rotates when the canonical cut cannot be made.
                let Some(added) = self.close_loop_with_fan(&loop_hes) else {
                    continue;
                };

                closed += 1;
                pass.progressed = true;
                progressed = true;
                pass.left_short_edge |= added.halfedge_ids.iter().any(|he_id| {
                    self.halfedges
                        .get(*he_id)
                        .is_some_and(|he| he.length_squared(self) < min_length_squared)
                });

                #[cfg(feature = "instrumentation")]
                crate::record_op_trace!(
                    "collapse hole closer: closed {loop_hes:?} with face {:?}",
                    added.face_id
                );
            }

            if !progressed {
                break;
            }
        }

        if closed > 0 {
            tracing::debug!("collapse closer closed {closed} boundary loops");
        }

        pass
    }

    /// Removes the wire edges a vertex of `candidates` belongs to: edges with no face on
    /// either side.
    ///
    /// Such an edge carries no surface - two face removals met on it and left both halves
    /// face-less - and it blocks every boundary walk, because a vertex of a wire has an
    /// extra outgoing boundary halfedge, which ends the walk. A hole that touches one
    /// cannot be closed until the wire is gone, which is why the caller alternates this
    /// with the closer.
    ///
    /// Returns the halfedges it removed (both halves per wire).
    fn remove_wire_edges(&mut self, candidates: &[VertexId]) -> Vec<HalfedgeId> {
        let mut wires: Vec<HalfedgeId> = Vec::new();

        for &v_id in candidates {
            let Some(out_hes) = self.outgoing_halfedges.get(v_id) else {
                continue;
            };

            for he_id in out_hes {
                let is_wire = self.halfedges.get(*he_id).is_some_and(|he| {
                    he.is_boundary()
                        && he
                            .twin
                            .and_then(|twin_id| self.halfedges.get(twin_id))
                            .is_some_and(|twin| twin.is_boundary())
                });

                // The twin reports the same edge; the liveness filter below drops the
                // duplicate once the first half has taken the pair with it.
                if is_wire {
                    wires.push(*he_id);
                }
            }
        }

        let wires: Vec<HalfedgeId> = wires
            .into_iter()
            .filter(|he_id| self.halfedges.contains_key(*he_id))
            .collect();

        let mut endpoints: Vec<VertexId> = Vec::new();
        for he_id in &wires {
            if let Some(he) = self.halfedges.get(*he_id) {
                endpoints.extend(he.start_vertex(self));
                endpoints.push(he.end_vertex);
            }
        }

        for he_id in &wires {
            self.remove_only_halfedge_and_twin(*he_id);
        }

        // `rebuild_outgoing_halfedges` ran before this, so the seeds still name the wires.
        // A vertex the wire was all that held on to has nothing left: drop it too, rather than
        // leave an edgeless vertex in the mesh (callers prune their marked sets against the
        // mesh afterwards).
        for v_id in endpoints {
            self.reseed_outgoing_if_dead(v_id);

            let isolated = self.vertices.contains_key(v_id)
                && self.outgoing_halfedges.get(v_id).is_none_or(|out_hes| {
                    out_hes
                        .iter()
                        .all(|he_id| !self.halfedges.contains_key(*he_id))
                });
            if isolated {
                self.remove_only_vertex(v_id);
            }
        }

        wires
    }

    /// The loop of *newly opened* boundary through `start_v`, when it is one this op can
    /// close.
    ///
    /// Every step follows the single outgoing boundary halfedge whose edge was not
    /// boundary before this op; a vertex with none, or with several, ends the walk. Old
    /// boundary is ignored rather than disqualifying: a hole the collapse opens can run
    /// through a vertex that also sits on a border which was already there, and that
    /// border is what the loop is allowed to touch - its edges are not part of the loop.
    ///
    /// Returns the loop's halfedges only when it closes without revisiting a vertex.
    /// Every halfedge it returns is one this op opened, which is what keeps the closer
    /// from ever sealing a border that is meant to be open - and why the loop may be of
    /// any length: a hole this op opened is this op's to close, however wide (the weld's
    /// small-gap openings are six and seven halfedges across). The walk itself bounds the
    /// length: it stops at a repeat or at a vertex with no further opened boundary.
    pub(crate) fn boundary_loop_from(
        &self,
        start_v: VertexId,
        boundary_before: &(impl BoundaryAtEntry + ?Sized),
    ) -> Option<Vec<HalfedgeId>> {
        let mut he_id = self.sole_new_boundary_out(start_v, boundary_before)?;
        let mut loop_hes = vec![he_id];
        let mut loop_verts = vec![start_v];

        loop {
            let end_v = self.halfedges.get(he_id)?.end_vertex;

            if end_v == start_v {
                // A face needs at least three halfedges to enclose anything.
                return (loop_hes.len() >= 3).then_some(loop_hes);
            }

            if loop_verts.contains(&end_v) {
                return None;
            }

            loop_verts.push(end_v);
            he_id = self.sole_new_boundary_out(end_v, boundary_before)?;
            loop_hes.push(he_id);
        }
    }

    /// The vertices of every boundary edge that is not in `boundary_before`: the
    /// endpoints of the boundary this op opened.
    ///
    /// A hole the op opened consists of such edges, so its vertices are all in here -
    /// which makes this the complete, and only, place a closer has to look.
    pub(crate) fn opened_boundary_vertices(
        &self,
        boundary_before: &HashSet<(VertexId, VertexId)>,
    ) -> Vec<VertexId> {
        vertices_of_pairs(self.opened_boundary_pairs(boundary_before))
    }

    /// The boundary edges that are not in `boundary_before`, sorted: the boundary this op
    /// opened, across the whole mesh.
    pub(crate) fn opened_boundary_pairs(
        &self,
        boundary_before: &HashSet<(VertexId, VertexId)>,
    ) -> Vec<(VertexId, VertexId)> {
        let mut opened: Vec<(VertexId, VertexId)> = self
            .boundary_edge_pairs()
            .into_iter()
            .filter(|pair| !boundary_before.contains(pair))
            .collect();
        // The pairs come out of a hash set; the walks start from their vertices in this
        // order, so give it one that depends on the mesh only.
        opened.sort_unstable();
        opened
    }

    /// [`Self::opened_boundary_pairs`] among the edges at `region`'s vertices only, for an
    /// op that knows where it worked: it costs the region's edges instead of a scan of the
    /// whole mesh.
    pub(crate) fn opened_boundary_pairs_around(
        &self,
        region: impl IntoIterator<Item = VertexId>,
        boundary_before: &impl BoundaryAtEntry,
    ) -> Vec<(VertexId, VertexId)> {
        let mut opened: Vec<(VertexId, VertexId)> = Vec::new();

        for v_id in region {
            let Some(out_he_ids) = self.outgoing_halfedges.get(v_id) else {
                continue;
            };
            for &he_id in out_he_ids {
                let Some(he) = self.halfedges.get(he_id) else {
                    continue;
                };
                // Either half: a border edge whose face-less half ends here is still an
                // edge at this vertex.
                let twin_open = he
                    .twin
                    .and_then(|twin_id| self.halfedges.get(twin_id))
                    .is_some_and(|twin| twin.is_boundary());
                if (he.is_boundary() || twin_open)
                    && !boundary_before.was_boundary(v_id, he.end_vertex)
                {
                    opened.push(edge_key(v_id, he.end_vertex));
                }
            }
        }

        opened.sort_unstable();
        opened.dedup();
        opened
    }
}

/// The vertices of `pairs`, each once, in pair order: where the closers start their walks.
pub(crate) fn vertices_of_pairs(pairs: Vec<(VertexId, VertexId)>) -> Vec<VertexId> {
    let mut seen = HashSet::default();
    let mut vertices = Vec::new();
    for (a, b) in pairs {
        for v_id in [a, b] {
            if seen.insert(v_id) {
                vertices.push(v_id);
            }
        }
    }

    vertices
}

impl MeshGraph {
    /// The outgoing boundary halfedge of a vertex whose edge is *not* in `boundary_before` —
    /// i.e. boundary this op opened — and the lowest id when the vertex has several.
    ///
    /// Several means the hole *pinches* there: two lobes of one hole, or two holes, meeting at a
    /// vertex. Taking one deterministically rather than giving up lets the closer close that lobe
    /// and leave the rest walkable. Refusing there is what left the weld's small-gap rims open —
    /// measured: nine opened halfedges with no walkable loop, because one vertex carried two.
    fn sole_new_boundary_out(
        &self,
        vertex_id: VertexId,
        boundary_before: &(impl BoundaryAtEntry + ?Sized),
    ) -> Option<HalfedgeId> {
        let mut found = None;

        for he_id in self.outgoing_halfedges.get(vertex_id)? {
            let Some(he) = self.halfedges.get(*he_id) else {
                continue;
            };
            let Some(start) = he.start_vertex(self) else {
                continue;
            };

            if he.face.is_some() || boundary_before.was_boundary(start, he.end_vertex) {
                continue;
            }

            if found.is_none() || he_id < found.as_ref().unwrap() {
                found = Some(*he_id);
            }
        }

        found
    }

    /// The wire edges of the mesh — both halves face-less — as unordered vertex pairs.
    fn wire_edge_pairs(&self) -> HashSet<(VertexId, VertexId)> {
        self.halfedges
            .iter()
            .filter(|(_, he)| {
                he.is_boundary()
                    && he
                        .twin
                        .and_then(|twin_id| self.halfedges.get(twin_id))
                        .is_some_and(|twin| twin.is_boundary())
            })
            .filter_map(|(_, he)| {
                he.start_vertex(self)
                    .map(|start| edge_key(start, he.end_vertex))
            })
            .collect()
    }

    /// The boundary of the mesh as unordered vertex pairs.
    pub(crate) fn boundary_edge_pairs(&self) -> HashSet<(VertexId, VertexId)> {
        self.halfedges
            .iter()
            .filter(|(_, he)| he.is_boundary())
            .filter_map(|(_, he)| {
                he.start_vertex(self)
                    .map(|start| edge_key(start, he.end_vertex))
            })
            .collect()
    }

    #[inline]
    pub fn can_collapse_edge(&mut self, halfedge_id: HalfedgeId) -> bool {
        self.can_collapse_edge_inner(halfedge_id).is_some()
    }

    #[instrument(skip(self))]
    pub fn can_collapse_edge_inner(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> Option<(HalfedgeId, VertexId, VertexId, Vec3)> {
        // TODO : consider boundary edge
        //
        //          end_vertex
        //  .            .            .
        // ( ) ◀─────── ( ) ───────▶ ( )
        //  '      3     '     2      '
        //            ╱ ▲ │ ╲
        //          4╱  │ │  ╲1
        //          ╱   │ │0  ╲
        //         ╱    │ │    ╲
        //        ▼     │ │     ▼
        //       .      │ │      .
        //      ( )   he│ │twin ( )
        //       '      │ │      '
        //        ▲     │ │     ▲
        //         ╲    │ │    ╱
        //          ╲  0│ │   ╱
        //          1╲  │ │  ╱4
        //            ╲ │ ▼ ╱
        //  .      2     .     3      .
        // ( ) ◀─────── ( ) ───────▶ ( )
        //  '            '            '
        //         start_vertex

        let he = self
            .halfedges
            .get(halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;
        let twin_id = he.twin.or_else(error_none!("Twin halfedge not found"))?;
        let twin = self
            .halfedges
            .get(twin_id)
            .or_else(error_none!("Twin halfedge not found"))?;

        let start_vertex_id = twin.end_vertex;
        self.vertices
            .get_mut(start_vertex_id)
            .or_else(error_none!("Start vertex not found"))?
            .outgoing_halfedge = Some(halfedge_id);

        let end_vertex_id = he.end_vertex;
        self.vertices
            .get_mut(end_vertex_id)
            .or_else(error_none!("End vertex not found"))?
            .outgoing_halfedge = Some(twin_id);

        let start_pos = self
            .positions
            .get(start_vertex_id)
            .or_else(error_none!("Start position not found"))?;

        let end_pos = self
            .positions
            .get(end_vertex_id)
            .or_else(error_none!("End position not found"))?;

        let center = (start_pos + end_pos) * 0.5;

        self.check_link_condition(halfedge_id, twin_id)?;
        self.check_inverted_faces(start_vertex_id, center)?;
        self.check_inverted_faces(end_vertex_id, center)?;

        Some((twin_id, start_vertex_id, end_vertex_id, center))
    }

    /// The link condition, for interior and boundary edges alike.
    ///
    /// Collapsing `(u, v)` may only merge the two stars when they share no
    /// neighbour other than the vertices opposite the edge in its own faces - two
    /// for an interior edge, one for a boundary edge. When they share more, the
    /// collapse glues together two parts of the surface that already touch along
    /// those neighbours, and `(u, w)` picks up a second copy: parallel edges whose
    /// free halves the neighborhood cleanup then removes, leaving an open slit.
    /// The geometric inversion guard does not see this - the faces it checks stay
    /// convex - so the condition has to be topological.
    ///
    /// A boundary edge gets the same shared-neighbour test, plus the standard
    /// boundary-preservation clause: two boundary vertices may only be merged
    /// along a boundary edge. Merging them across the interior identifies two
    /// distinct border points and pinches the border, which turns one boundary
    /// loop into two that touch at a vertex.
    fn check_link_condition(&self, halfedge_id: HalfedgeId, twin_id: HalfedgeId) -> Option<()> {
        let he = self.halfedges.get(halfedge_id)?;
        let twin = self.halfedges.get(twin_id)?;

        let start_v = twin.end_vertex;
        let end_v = he.end_vertex;

        // `link(uv)`: the vertices opposite the edge in its incident faces. A
        // boundary half has no `next`, so `opposite_vertex` cannot answer for it.
        let mut allowed: Vec<VertexId> = Vec::with_capacity(2);
        for h in [he, twin] {
            if h.is_boundary() {
                continue;
            }
            let opposite = h.opposite_vertex(self)?;
            if !allowed.contains(&opposite) {
                allowed.push(opposite);
            }
        }

        // Boundary preservation: only a boundary edge may merge two boundary
        // vertices.
        if !he.is_boundary()
            && !twin.is_boundary()
            && self.has_boundary_halfedge(start_v)
            && self.has_boundary_halfedge(end_v)
        {
            return None;
        }

        let start_ring = self.ring_end_vertices(start_v);
        for v in self.ring_end_vertices(end_v) {
            if v == start_v || allowed.contains(&v) {
                continue;
            }
            if start_ring.contains(&v) {
                return None;
            }
        }

        Some(())
    }

    /// Whether any halfedge leaving `v` is face-less.
    ///
    /// Scans the raw outgoing list rather than [`Vertex::is_boundary`], whose
    /// single seed halfedge can sit on an interior fan of a pinched vertex.
    fn has_boundary_halfedge(&self, v: VertexId) -> bool {
        self.outgoing_halfedges.get(v).is_some_and(|out| {
            out.iter()
                .any(|h| self.halfedges.get(*h).is_some_and(|h| h.is_boundary()))
        })
    }

    /// The end vertices of every halfedge leaving `v`, in list order (duplicates
    /// included - a pinched star reaches the same neighbour from more than one
    /// fan).
    fn ring_end_vertices(&self, v: VertexId) -> Vec<VertexId> {
        self.outgoing_halfedges
            .get(v)
            .map(|out| {
                out.iter()
                    .filter_map(|h| self.halfedges.get(*h))
                    .map(|h| h.end_vertex)
                    .collect()
            })
            .unwrap_or_default()
    }

    fn check_inverted_faces(&self, vertex_id: VertexId, center: Vec3) -> Option<()> {
        // just made sure that this exists
        let face_ids = self.vertices[vertex_id].faces(self).skip(2).collect_vec();

        for face_id in face_ids {
            let mut orig_positions = Vec::with_capacity(3);
            let mut new_positions = Vec::with_capacity(3);

            let face = self
                .faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?;

            for v_id in face.vertices(self) {
                let pos = *self
                    .positions
                    .get(v_id)
                    .or_else(error_none!("Vertex pos not found"))?;

                if v_id == vertex_id {
                    new_positions.push(center);
                } else {
                    new_positions.push(pos);
                }
                orig_positions.push(pos);
            }

            let Some(orig_normal) = Face::normal_from_positions(&orig_positions) else {
                continue;
            };
            let Some(new_normal) = Face::normal_from_positions(&new_positions) else {
                continue;
            };

            if orig_normal.dot(new_normal) < 0.0 {
                return None;
            }
        }

        Some(())
    }

    #[instrument(skip(self))]
    pub fn collapse_edge_inner(
        &mut self,
        halfedge_id: HalfedgeId,
        twin_id: HalfedgeId,
        start_v_id: VertexId,
        end_v_id: VertexId,
        center_pos: Vec3,
    ) -> CollapseEdge {
        let mut result = CollapseEdge::default();

        if start_v_id == end_v_id {
            error!("Cannot collapse edge between the same vertex");
            return result;
        }

        // #[cfg(feature = "rerun")]
        // {
        //     self.log_he_rerun("collapse/he", halfedge_id);
        // }
        // TODO : consider border vertices

        let he = *unwrap_or_return!(
            self.halfedges.get(halfedge_id),
            "Halfedge not found",
            result
        );
        let twin = *unwrap_or_return!(
            self.halfedges.get(twin_id),
            "Twin halfedge not found",
            result
        );

        if !he.is_boundary() {
            let (face_id, halfedge_ids) = unwrap_or_return!(
                self.remove_halfedge_face(halfedge_id),
                "Could not remove face",
                result
            );

            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(halfedge_ids);
        }
        result.removed_halfedges.push(halfedge_id);

        self.remove_outgoing_halfedge(start_v_id, halfedge_id);

        if !twin.is_boundary() {
            let twin_face_removal = self.remove_halfedge_face(twin_id);
            #[cfg(feature = "instrumentation")]
            crate::record_op_trace!(
                "collapse_edge_inner({halfedge_id:?}): twin-side face removal {twin_face_removal:?}"
            );
            if twin_face_removal.is_none() {
                // The twin-side dismantling failed — typically because the twin was
                // already removed by the start-side dismantling (a degenerate fold
                // whose start-side face chain contains the collapsed edge's own
                // twin). Aborting right away would strand `halfedge_id` with a face
                // pointer to the already-removed start-side face (the layer-4 ghost:
                // a live halfedge claiming a removed face). Heal the survivor first:
                // detach it from the dead face and re-pair it with a fresh boundary
                // half so no invariant is violated when the op terminates.
                if let Some(he_mut) = self.halfedges.get_mut(halfedge_id) {
                    he_mut.face = None;
                    he_mut.next = None;
                }
                self.pair_with_fresh_boundary_half(halfedge_id, start_v_id);
                return result;
            }
            let (face_id, halfedge_ids) =
                unwrap_or_return!(twin_face_removal, "Failed to remove halfedge face", result);

            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(halfedge_ids);
        }
        result.removed_halfedges.push(twin_id);

        self.remove_outgoing_halfedge(end_v_id, twin_id);

        // Remove the collapsed edge's own halfedges now. Their twins are each
        // other, so both partners go in the same batch: nothing survives with a
        // reference to them, no re-pairing needed.
        #[cfg(feature = "instrumentation")]
        self.probe_live_face_removal(&[halfedge_id, twin_id], "collapse_own");
        self.halfedges.remove(halfedge_id);
        self.halfedges.remove(twin_id);

        // The end vertex is absorbed into the start vertex. Every surviving halfedge that
        // still points at the end vertex must be re-pointed at the start vertex, and the end
        // vertex's outgoing halfedges move onto the start vertex's list. `remove_halfedge_face`
        // keeps the outgoing lists (including the re-paired twins) in sync, so the end
        // vertex's list is complete here.
        let end_outgoing = self
            .outgoing_halfedges
            .get(end_v_id)
            .cloned()
            .unwrap_or_default();
        for &out_he_id in &end_outgoing {
            let Some(out_he) = self.halfedges.get(out_he_id) else {
                continue;
            };
            let Some(twin_id) = out_he.twin else {
                continue;
            };
            if let Some(twin) = self.halfedges.get_mut(twin_id) {
                twin.end_vertex = start_v_id;
            }
        }
        self.outgoing_halfedges
            .entry(start_v_id)
            .unwrap() // key exists because we access positions[start_v_id] below
            .or_default()
            .extend(end_outgoing);

        self.remove_only_vertex(end_v_id);
        result.removed_vertices.push(end_v_id);

        // key exists. we accessed it above
        self.positions[start_v_id] = center_pos;

        // Pick a surviving outgoing halfedge of the start vertex as its seed pointer.
        let new_outgoing_he_id = self.outgoing_halfedges.get(start_v_id).and_then(|l| {
            l.iter()
                .copied()
                .find(|he_id| self.halfedges.contains_key(*he_id))
        });

        if let Some(new_outgoing_he_id) = new_outgoing_he_id {
            // key exists. we accessed it above
            self.vertices[start_v_id].outgoing_halfedge = Some(new_outgoing_he_id);

            // #[cfg(feature = "rerun")]
            // {
            //     self.log_he_rerun(
            //         "collapse/outgoing",
            //         self.vertices[start_v_id].outgoing_halfedge.unwrap(),
            //     );
            // }

            let cleanup = self.make_vertex_neighborhood_manifold(start_v_id);

            result.added_vertices = cleanup.added_vertices;
            result.removed_vertices.extend(cleanup.removed_vertices);
            result.removed_halfedges.extend(cleanup.removed_halfedges);
            result.removed_faces.extend(cleanup.removed_faces);
        } else {
            self.remove_only_vertex(start_v_id);

            result.removed_vertices.push(start_v_id);
        }

        result
    }

    /// Collapse an edge in the mesh graph.
    ///
    /// This moves the start vertex of the edge to the center of the edge
    /// and removes the end vertex and the adjacent and opposite faces.
    ///
    /// It also performs a cleanup afterwards to remove flaps (faces that share the same vertices).
    ///
    /// Returns the vertices, halfedges and faces that were removed.
    #[instrument(skip(self))]
    pub fn collapse_edge(&mut self, halfedge_id: HalfedgeId) -> CollapseEdge {
        let he = *unwrap_or_return!(
            self.halfedges.get(halfedge_id),
            "Halfedge not found",
            CollapseEdge::default()
        );
        let twin_id = unwrap_or_return!(he.twin, "Twin missing", CollapseEdge::default());
        let twin = unwrap_or_return!(
            self.halfedges.get(twin_id),
            "Halfedge not found",
            CollapseEdge::default()
        );

        let start_v_id = twin.end_vertex;
        let end_v_id = he.end_vertex;

        let start_pos = *unwrap_or_return!(
            self.positions.get(start_v_id),
            "Start position not found",
            CollapseEdge::default()
        );
        let end_pos = *unwrap_or_return!(
            self.positions.get(end_v_id),
            "End position not found",
            CollapseEdge::default()
        );

        let center_pos = (start_pos + end_pos) * 0.5;

        self.collapse_edge_inner(halfedge_id, twin_id, start_v_id, end_v_id, center_pos)
    }

    /// Remove a halfedge face and re-connecting the adjacent halfedges.
    /// Only works on manifold triangle meshes.
    #[instrument(skip(self))]
    fn remove_halfedge_face(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> Option<(FaceId, [HalfedgeId; 2])> {
        let he = self
            .halfedges
            .get(halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;

        let face_id = he.face.or_else(error_none!("Face not found"))?;

        let next_he_id = he.next.or_else(error_none!("Next halfedge is None"))?;
        let prev_he_id = he
            .prev(self)
            .or_else(error_none!("Previous halfedge is None"))?;

        let next_twin_id = self
            .halfedges
            .get(next_he_id)
            .or_else(error_none!("Next halfedge not found"))?
            .twin
            .or_else(error_none!("Next twin halfedge not found"))?;
        let prev_twin_id = self
            .halfedges
            .get(prev_he_id)
            .or_else(error_none!("Previous halfedge not found"))?
            .twin
            .or_else(error_none!("Previous twin halfedge not found"))?;

        let next_he = self
            .halfedges
            .get(next_he_id)
            .or_else(error_none!("Next halfedge not found"))?;
        let prev_he = self
            .halfedges
            .get(prev_he_id)
            .or_else(error_none!("Previous halfedge not found"))?;

        let next_end_v_id = next_he.end_vertex;
        let prev_end_v_id = prev_he.end_vertex;

        // A pre-existing asymmetric twin
        // pair (e.g. from a flap in a degenerate neighborhood) can make the face-derived
        // vertices above disagree with the derived starts; removing from the face-derived
        // vertex then leaves a stale entry behind.
        let next_he_derived_start = next_he
            .start_vertex(self)
            .or_else(error_none!("Next halfedge start vertex not found"))?;
        let prev_he_derived_start = prev_he
            .start_vertex(self)
            .or_else(error_none!("Previous halfedge start vertex not found"))?;

        // `prev_twin`'s and `next_twin`'s derived starts before the re-linking below:
        // `twin(he).end`. In the symmetric case these are `prev_he.end_vertex` and
        // `next_he.end_vertex`; with an asymmetric twin pair (or a flap where both faces of
        // the collapsed edge share all three vertices) they can differ, so derive them from
        // the topology rather than the face. Must be computed before `next_he`/`prev_he` are
        // deleted below: they are the twins whose ends define these starts.
        let prev_twin_old_start = self
            .halfedges
            .get(prev_twin_id)
            .and_then(|he| he.twin)
            .and_then(|twin_id| self.halfedges.get(twin_id))
            .or_else(error_none!("Previous twin start vertex not found"))?
            .end_vertex;
        let next_twin_old_start = self
            .halfedges
            .get(next_twin_id)
            .and_then(|he| he.twin)
            .and_then(|twin_id| self.halfedges.get(twin_id))
            .or_else(error_none!("Next twin start vertex not found"))?
            .end_vertex;

        self.vertices
            .get_mut(next_end_v_id)
            .or_else(error_none!("Next end vertex not found"))?
            .outgoing_halfedge = next_he.next.or_else(error_none!("Next next is None"));
        self.vertices
            .get_mut(prev_end_v_id)
            .or_else(error_none!("Previous end vertex not found"))?
            .outgoing_halfedge = prev_he.next.or_else(error_none!("Previous next is None"));

        let prev_start_v_id = prev_he
            .start_vertex(self)
            .or_else(error_none!("Previous start vertex ID not found"))?;
        let prev_start_v = self
            .vertices
            .get(prev_start_v_id)
            .or_else(error_none!("Previous start vertex not found"))?;

        if prev_start_v.outgoing_halfedge == Some(prev_he_id) {
            // checked just above
            self.vertices[prev_start_v_id].outgoing_halfedge = prev_he
                .ccw_rotated_neighbour(self)
                .or_else(|| prev_he.cw_rotated_neighbour(self))
                .or_else(error_none!(
                    "Previous start vertex new outgoing halfedge not found"
                ));
        }

        self.bvh.remove(
            self.faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?
                .index,
        );

        #[cfg(feature = "instrumentation")]
        self.probe_live_face_removal(&[next_he_id, prev_he_id], "remove_halfedge_face");
        self.halfedges.remove(next_he_id);
        self.halfedges.remove(prev_he_id);
        self.remove_outgoing_halfedge(next_he_derived_start, next_he_id);
        self.remove_outgoing_halfedge(prev_he_derived_start, prev_he_id);

        if let Some(face) = self.faces.remove(face_id) {
            self.bvh.remove(face.index);
        }
        #[cfg(feature = "instrumentation")]
        crate::record_face_death(face_id);

        if self.halfedges.contains_key(next_twin_id) && self.halfedges.contains_key(prev_twin_id) {
            self.halfedges.get_mut(next_twin_id).unwrap().twin = Some(prev_twin_id);
            self.halfedges.get_mut(prev_twin_id).unwrap().twin = Some(next_twin_id);

            // Re-linking the twins above changes the *derived* start vertex of `prev_twin`
            // (`start_vertex == twin.end_vertex`): it used to start at `prev_he.end_vertex` and
            // now starts at `next_twin.end_vertex`. Keep `outgoing_halfedges` consistent by moving
            // `prev_twin` between the two lists.
            let prev_twin_new_start = self
                .halfedges
                .get(next_twin_id)
                .or_else(error_none!("Next twin halfedge not found"))?
                .end_vertex;
            self.remove_outgoing_halfedge(prev_twin_old_start, prev_twin_id);
            if let Some(list) = self.outgoing_halfedges.get_mut(prev_twin_new_start) {
                list.push(prev_twin_id);
            }

            // The re-link above may also change `next_twin`'s derived start (in a flap, where
            // `prev_twin.end_vertex` differs from `next_he.end_vertex`). Keep its list entry in
            // sync like `prev_twin`'s: `next_twin`'s new derived start is `prev_twin.end_vertex`.
            let next_twin_new_start = self
                .halfedges
                .get(prev_twin_id)
                .or_else(error_none!("Previous twin halfedge not found"))?
                .end_vertex;
            if next_twin_old_start != next_twin_new_start {
                self.remove_outgoing_halfedge(next_twin_old_start, next_twin_id);
                if let Some(list) = self.outgoing_halfedges.get_mut(next_twin_new_start) {
                    list.push(next_twin_id);
                }
            }
        } else {
            // One or both twins are gone (degenerate neighborhood where a twin was
            // another halfedge of the same removed face, or a self-twinned member).
            // Whichever twin partner survives must not be left twinless: re-pair it
            // with a fresh boundary half so the invariant (every halfedge has a
            // twin) holds when the op terminates.
            if self.halfedges.contains_key(next_twin_id)
                && self
                    .pair_with_fresh_boundary_half(next_twin_id, next_end_v_id)
                    .is_none()
            {
                error!("remove_halfedge_face: could not re-pair next twin {next_twin_id:?}");
            }
            if self.halfedges.contains_key(prev_twin_id)
                && prev_twin_id != next_twin_id
                && self
                    .pair_with_fresh_boundary_half(prev_twin_id, prev_end_v_id)
                    .is_none()
            {
                error!("remove_halfedge_face: could not re-pair prev twin {prev_twin_id:?}");
            }
        }

        Some((face_id, [next_he_id, prev_he_id]))
    }
}

#[derive(Default, Debug)]
pub struct CollapseEdge {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    pub added_vertices: Vec<VertexId>,
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::ops::EdgeLengthCleanup;
    use crate::utils::{build_grid, mesh_invariant_violations};

    #[test]
    #[allow(unused_variables)]
    fn test_collapse_edge() {
        let mut mesh_graph = MeshGraph::new();

        let face1 = mesh_graph
            .add_face_from_positions(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 4.0, 0.0),
                Vec3::new(1.0, 2.0, 0.0),
            )
            .face_id;

        let he1 = mesh_graph.faces[face1]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[1];

        let face2 = mesh_graph
            .add_face_from_halfedge_and_position(he1, Vec3::new(2.0, 4.0, 0.0))
            .unwrap()
            .face_id;

        let he2 = mesh_graph.faces[face2]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[2];

        let face3 = mesh_graph
            .add_face_from_halfedge_and_position(he2, Vec3::new(3.0, 2.0, 0.0))
            .unwrap()
            .face_id;

        let he3 = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[1];

        let face4 = mesh_graph
            .add_face_from_halfedge_and_position(he3, Vec3::new(4.0, 4.0, 0.0))
            .unwrap()
            .face_id;

        let he4 = mesh_graph.faces[face4]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face5 = mesh_graph
            .add_face_from_halfedge_and_position(he4, Vec3::new(4.0, 0.0, 0.0))
            .unwrap()
            .face_id;

        let he5 = mesh_graph.faces[face5]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face6 = mesh_graph
            .add_face_from_halfedge_and_position(he5, Vec3::new(2.0, 0.0, 0.0))
            .unwrap()
            .face_id;

        let he6 = mesh_graph.faces[face6]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let he3 = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face7 = mesh_graph
            .add_face_from_halfedges(he6, he3)
            .unwrap()
            .face_id;

        let he7 = mesh_graph.faces[face7]
            .halfedges(&mesh_graph)
            .next()
            .unwrap();

        let he1 = mesh_graph.faces[face1]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        mesh_graph.add_face_from_halfedges(he1, he7).unwrap();

        assert_eq!(mesh_graph.vertices.len(), 8);
        assert_eq!(mesh_graph.halfedges.len(), 30);
        assert_eq!(mesh_graph.faces.len(), 8);

        #[cfg(feature = "rerun")]
        mesh_graph.log_rerun();

        let edge_to_collapse = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let start_v_id = mesh_graph.halfedges[edge_to_collapse]
            .start_vertex(&mesh_graph)
            .unwrap();

        assert_eq!(mesh_graph.outgoing_halfedges[start_v_id].len(), 5);

        let CollapseEdge {
            removed_vertices,
            removed_halfedges,
            removed_faces,
            added_vertices,
        } = mesh_graph.collapse_edge(edge_to_collapse);

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        assert_eq!(removed_vertices.len(), 1);
        assert_eq!(removed_halfedges.len(), 6);
        assert_eq!(removed_faces.len(), 2);

        assert_eq!(mesh_graph.vertices.len(), 7);
        assert_eq!(mesh_graph.halfedges.len(), 24);
        assert_eq!(mesh_graph.faces.len(), 6);

        assert_eq!(mesh_graph.outgoing_halfedges[start_v_id].len(), 6);
    }

    /// An edge the inversion guard permanently refuses leaves the mesh dirty, and the
    /// op must say so rather than letting the caller assume it finished.
    #[test]
    fn test_collapse_reports_stalled_when_an_edge_cannot_collapse() {
        const MIN_LEN_SQR: f32 = 0.2;

        let mut mg = build_grid(4);

        let vertex_at = |mg: &MeshGraph, x: f32, y: f32| -> VertexId {
            mg.positions
                .iter()
                .find(|(_, p)| (p.x - x).abs() < 1e-6 && (p.y - y).abs() < 1e-6)
                .map(|(v, _)| v)
                .expect("grid has no vertex at that position")
        };

        let v = vertex_at(&mg, 2.0, 2.0);
        let w = vertex_at(&mg, 1.0, 2.0);
        mg.positions[v] = Vec3::new(2.5, 2.95, 0.0);
        mg.positions[w] = Vec3::new(2.5, 3.1, 0.0);
        mg.compute_vertex_normals();

        let outcome =
            mg.collapse_until_edges_above_min_length(MIN_LEN_SQR, &mut HashSet::default());

        assert_eq!(outcome, EdgeLengthCleanup::Stalled);
        assert!(mesh_invariant_violations(&mg).is_empty());
    }

    #[test]
    fn test_collapse_reports_converged_when_it_drains() {
        let mut mg = build_grid(6);

        let outcome = mg.collapse_until_edges_above_min_length(2.0, &mut HashSet::default());

        assert_eq!(outcome, EdgeLengthCleanup::Converged);
        assert_eq!(
            mg.halfedges
                .values()
                .filter(|he| he.length_squared(&mg) < 2.0)
                .count(),
            0
        );
    }

    /// A mesh with nothing below the threshold converges without touching anything.
    #[test]
    fn test_collapse_reports_converged_on_a_clean_mesh() {
        let mut mg = build_grid(3);
        let faces = mg.faces.len();

        let outcome = mg.collapse_until_edges_above_min_length(0.01, &mut HashSet::default());

        assert_eq!(outcome, EdgeLengthCleanup::Converged);
        assert_eq!(mg.faces.len(), faces);
    }

    #[test]
    fn test_collapse_until_min_length_leaves_no_dangling_halfedges() {
        let mut mg = build_grid(8);
        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "initial mesh must satisfy invariants"
        );

        let mut marked = HashSet::default();
        mg.collapse_until_edges_above_min_length(2.0, &mut marked);

        let problems = mesh_invariant_violations(&mg);
        assert!(
            problems.is_empty(),
            "collapse left {} dangling/inconsistent halfedges, e.g.:\n{}\n",
            problems.len(),
            problems.iter().take(10).join("\n")
        );
    }

    /// The above asserts only that the mesh stayed well formed, which a collapse
    /// that picked the wrong edges — or no edges — would also satisfy. These pin
    /// that work actually happened and moved in the right direction.
    #[test]
    fn test_collapse_until_min_length_removes_short_edges() {
        let mut mg = build_grid(6);

        let below_before = mg
            .halfedges
            .values()
            .filter(|he| he.length_squared(&mg) < 2.0)
            .count();
        let faces_before = mg.faces.len();
        assert!(below_before > 0, "fixture has no edges below the threshold");

        mg.collapse_until_edges_above_min_length(2.0, &mut HashSet::default());

        assert!(mesh_invariant_violations(&mg).is_empty());

        let below_after = mg
            .halfedges
            .values()
            .filter(|he| he.length_squared(&mg) < 2.0)
            .count();

        assert!(
            below_after < below_before,
            "collapse made no progress: {below_before} -> {below_after} short edges"
        );
        assert!(
            mg.faces.len() < faces_before,
            "collapsing edges must remove faces: {faces_before} -> {}",
            mg.faces.len()
        );
    }

    /// Aggressive collapse on non-planar geometry stays well formed.
    ///
    /// Note on what this does *not* cover: the deferred-requeue path, which holds
    /// candidates that `can_collapse_edge_inner` rejected. Rejections come from
    /// `check_inverted_faces`, and measurement shows this fixture produces zero
    /// of them even with the ridges — as does the flat grid, because collapsing to
    /// a midpoint inside a convex one-ring cannot flip a face normal. That path is
    /// covered by `test_collapse_retries_candidates_the_inversion_guard_rejected`
    /// below, which builds the non-convex case on purpose, and at the unit level by
    /// `ops::pending_edges_test::requeue_makes_a_declined_entry_reachable_again`;
    /// on production scans it fires 64-18466 times per pass.
    #[test]
    fn test_collapse_until_min_length_on_non_planar_grid() {
        let mut mg = build_grid(6);

        let displaced: Vec<(VertexId, Vec3)> = mg
            .positions
            .iter()
            .map(|(v, &p)| {
                let ridge = ((p.x as i32 % 2) ^ (p.y as i32 % 2)) as f32;
                (v, Vec3::new(p.x, p.y, ridge * 1.5))
            })
            .collect();
        for (v, p) in displaced {
            mg.positions[v] = p;
        }
        mg.compute_vertex_normals();

        let faces_before = mg.faces.len();
        mg.collapse_until_edges_above_min_length(3.0, &mut HashSet::default());

        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "invariants violated: {:?}",
            mesh_invariant_violations(&mg)
        );
        assert!(
            mg.faces.len() < faces_before,
            "collapse made no progress on the ridged grid"
        );
    }

    /// The deferred-requeue path: a candidate `can_collapse_edge_inner` rejected is
    /// pushed back onto the heap after the next successful collapse, instead of being
    /// dropped. `pop_live` consumes the heap entry without touching the map, so a
    /// rejected id that is not requeued stays pending with nothing pointing at it —
    /// unreachable for the rest of the run.
    ///
    /// Reaching a rejection at all takes deliberate construction; the grids above
    /// produce none. `check_inverted_faces` rejects only when moving a vertex to the
    /// edge midpoint flips a face normal, which needs the midpoint to land across the
    /// line through two of that vertex's *other* ring neighbours — impossible inside
    /// a convex one-ring. So `v` is pulled to just inside the line `y = 3` through its
    /// neighbours `(3,3)` and `(2,3)`, and `w` is pushed just across it. Edge `v-w` is
    /// len_sqr 0.0225, the shortest in the mesh, so it is popped first, and its
    /// midpoint at `y = 3.025` inverts the face `(v, (3,3), (2,3))`.
    ///
    /// The second edit is what makes the requeue observable: a collapsible short edge
    /// in the far corner, `len_sqr` 0.09. The run then goes reject `v-w` → collapse
    /// the corner edge → retry `v-w`. The corner collapse is far enough away that
    /// `v-w` is not in its `halfedges_to_check`, so the re-check pass cannot re-insert
    /// it; only the requeue can bring it back. Drop the requeue and the final
    /// `debug_assert!` in the op sees one pending edge and an empty `deferred`.
    #[test]
    fn test_collapse_retries_candidates_the_inversion_guard_rejected() {
        const MIN_LEN_SQR: f32 = 0.2;

        let mut mg = build_grid(4);

        let vertex_at = |mg: &MeshGraph, x: f32, y: f32| -> VertexId {
            mg.positions
                .iter()
                .find(|(_, p)| (p.x - x).abs() < 1e-6 && (p.y - y).abs() < 1e-6)
                .map(|(v, _)| v)
                .expect("grid has no vertex at that position")
        };

        let v = vertex_at(&mg, 2.0, 2.0);
        let w = vertex_at(&mg, 1.0, 2.0);
        let corner = vertex_at(&mg, 0.0, 0.0);
        mg.positions[v] = Vec3::new(2.5, 2.95, 0.0);
        mg.positions[w] = Vec3::new(2.5, 3.1, 0.0);
        mg.positions[corner] = Vec3::new(0.7, 0.0, 0.0);
        mg.compute_vertex_normals();

        let below = |mg: &MeshGraph| {
            mg.halfedges
                .values()
                .filter(|he| he.length_squared(mg) < MIN_LEN_SQR)
                .count()
        };

        // Exactly two edges pending: the inverting `v-w` and the corner edge.
        assert_eq!(below(&mg), 4, "fixture should seed exactly two short edges");
        let faces_before = mg.faces.len();

        mg.collapse_until_edges_above_min_length(MIN_LEN_SQR, &mut HashSet::default());

        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "invariants violated: {:?}",
            mesh_invariant_violations(&mg)
        );
        // The rejected candidate sits at the front of the queue; the op must still
        // get past it and collapse the corner edge.
        assert!(
            mg.faces.len() < faces_before,
            "a rejected shortest edge stalled the whole op: {faces_before} -> {} faces",
            mg.faces.len()
        );
        // `v-w` inverts a face no later collapse repairs, so it stays pending - and
        // stays *retried*, which is the half a dropped requeue would lose.
        assert_eq!(
            below(&mg),
            2,
            "expected only the inverting edge to survive, found {} short halfedges",
            below(&mg)
        );
    }

    #[cfg(feature = "gltf")]
    #[test]
    fn test_can_collapse_edge() {
        use crate::{integrations::gltf, utils::get_tracing_subscriber};

        get_tracing_subscriber();
        let mut meshgraph = gltf::load("src/ops/glb/can_collapse_edge.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let mut v_top_id = VertexId::default();
        let mut v_bottom_id = VertexId::default();

        for (v_id, pos) in &meshgraph.positions {
            if pos.x == 0.0 {
                if pos.y == -1.0 {
                    v_top_id = v_id;
                } else if pos.y == 1.0 {
                    v_bottom_id = v_id;
                }
            }
        }

        if v_top_id == VertexId::default() {
            panic!("No top vertex found");
        }

        if v_bottom_id == VertexId::default() {
            panic!("No bottom vertex found");
        }

        let he_id = meshgraph.halfedge_from_to(v_top_id, v_bottom_id).unwrap();

        let result = meshgraph.can_collapse_edge(he_id);

        assert!(result);

        #[cfg(feature = "rerun")]
        crate::RR.flush_blocking().unwrap();
    }

    #[cfg(feature = "gltf")]
    #[test]
    fn test_cannot_collapse_edge() {
        use crate::{integrations::gltf, utils::get_tracing_subscriber};

        get_tracing_subscriber();
        let mut meshgraph = gltf::load("src/ops/glb/cannot_collapse_edge.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let mut v_top_id = VertexId::default();
        let mut v_bottom_id = VertexId::default();

        for (v_id, pos) in &meshgraph.positions {
            if pos.x == 0.0 {
                if pos.y == -1.0 {
                    v_top_id = v_id;
                } else if pos.y == 1.0 {
                    v_bottom_id = v_id;
                }
            }
        }

        if v_top_id == VertexId::default() {
            panic!("No top vertex found");
        }

        if v_bottom_id == VertexId::default() {
            panic!("No bottom vertex found");
        }

        let he_id = meshgraph.halfedge_from_to(v_top_id, v_bottom_id).unwrap();

        let result = meshgraph.can_collapse_edge(he_id);

        assert!(!result);

        #[cfg(feature = "rerun")]
        crate::RR.flush_blocking().unwrap();
    }

    /// Collapse absorbs one endpoint into the other, so ids the caller marked can name
    /// vertices that no longer exist by the time it returns. `marked_vertices` is also
    /// extended with the vertices the cleanup creates, so a caller that only ever adds
    /// to it never notices unless the dead ones are pruned too.
    #[test]
    fn test_collapse_purges_dead_ids_from_marked_vertices() {
        let mut mg = build_grid(4);
        let mut marked: HashSet<VertexId> = mg.vertices.keys().collect();
        let before = marked.len();

        mg.collapse_until_edges_above_min_length(1.5, &mut marked);

        assert!(
            marked.len() < before,
            "no vertex was collapsed - test is vacuous"
        );
        for v_id in &marked {
            assert!(
                mg.vertices.contains_key(*v_id),
                "marked vertex {v_id:?} is dead"
            );
        }
    }
}

/// The link condition: an edge whose endpoints share a neighbour that is not one
/// of the vertices opposite it must not be collapsible.
///
/// The fixture is the one-ring of such an edge, extracted from a freestyle weld
/// that opened a two-halfedge slit. The weld's cleanup collapsed this edge,
/// because the guard only checked for inverted faces: the collapse left a second
/// copy of `(u, w)` behind, and removing the resulting coincident faces left the
/// surface open. The gate must reject it on topology alone.
#[cfg(test)]
mod link_condition_tests {
    use crate::{MeshGraph, Vec3};

    #[test]
    fn test_edge_breaking_the_link_condition_is_not_collapsible() {
        let positions = [
            Vec3::new(1.5474508, 0.8314448, 0.48144284),
            Vec3::new(1.476615, 0.7591524, 0.5624269),
            Vec3::new(1.5139911, 0.9200293, 0.5236689),
            Vec3::new(1.48824, 0.9244807, 0.68454176),
            Vec3::new(1.4824275, 0.84181654, 0.6234844),
            Vec3::new(1.5372095, 1.0181688, 0.64666533),
            Vec3::new(1.4763294, 0.9111748, 0.5377056),
            Vec3::new(1.5632365, 0.898358, 0.37499934),
            Vec3::new(1.4542371, 1.0170964, 0.42268354),
            Vec3::new(1.5412235, 1.0512308, 0.41721624),
            Vec3::new(1.4374998, 0.75037587, 0.37499997),
            Vec3::new(1.4456218, 0.9199032, 0.3959853),
            Vec3::new(1.4714506, 1.0471027, 0.6503712),
        ];
        let indices = [
            0, 1, 2, 3, 2, 4, 5, 6, 2, 4, 2, 1, 2, 7, 0, 3, 5, 2, 2, 8, 9, 7, 2, 9, 8, 2, 6, 6, 10,
            11, 6, 1, 10, 4, 6, 3, 3, 6, 12, 1, 6, 4, 6, 5, 12, 11, 8, 6,
        ];

        let (mut mesh, vertex_ids) =
            MeshGraph::indexed_triangles_and_vertex_ids(&positions, &indices);
        let start = vertex_ids[2];
        let end = vertex_ids[6];
        let halfedge_id = mesh
            .halfedges_from_to(start, end)
            .into_iter()
            .next()
            .expect("edge 2 -> 6 must exist");

        // The condition only applies to an edge with two adjacent faces.
        let twin_id = mesh.halfedges[halfedge_id].twin.expect("twin");
        assert!(mesh.halfedges[halfedge_id].face.is_some());
        assert!(mesh.halfedges[twin_id].face.is_some());

        assert!(
            !mesh.can_collapse_edge(halfedge_id),
            "an edge whose endpoints share five neighbours must not be collapsible"
        );
    }

    /// A quad, so the interior diagonal joins two boundary vertices: merging them
    /// would pinch the border. The link condition alone passes here - the two
    /// stars share exactly the two vertices opposite the diagonal - so this pins
    /// the boundary-preservation clause.
    #[test]
    fn test_interior_edge_between_boundary_vertices_is_not_collapsible() {
        let positions = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.2),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.2),
        ];
        // (0, 1, 2) and (0, 2, 3): the diagonal 0-2 is interior.
        let indices = [0, 1, 2, 0, 2, 3];
        let (mut mesh, ids) = MeshGraph::indexed_triangles_and_vertex_ids(&positions, &indices);

        let diagonal = mesh
            .halfedges_from_to(ids[0], ids[2])
            .into_iter()
            .next()
            .expect("diagonal must exist");
        let twin = mesh.halfedges[diagonal].twin.expect("twin");
        assert!(mesh.halfedges[diagonal].face.is_some());
        assert!(mesh.halfedges[twin].face.is_some());

        assert!(
            !mesh.can_collapse_edge(diagonal),
            "merging two boundary vertices across the interior would pinch the border"
        );
    }

    /// The same quad's border edge must stay collapsible: it is a boundary edge,
    /// so merging its two boundary endpoints runs along the border.
    #[test]
    fn test_boundary_edge_stays_collapsible() {
        let positions = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.2),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.2),
        ];
        let indices = [0, 1, 2, 0, 2, 3];
        let (mut mesh, ids) = MeshGraph::indexed_triangles_and_vertex_ids(&positions, &indices);

        let border = mesh
            .halfedges
            .iter()
            .find_map(|(he_id, he)| {
                (he.is_boundary()
                    && he.end_vertex == ids[1]
                    && he.start_vertex(&mesh) == Some(ids[0]))
                .then_some(he_id)
            })
            .or_else(|| {
                mesh.halfedges.iter().find_map(|(he_id, he)| {
                    (he.is_boundary()
                        && he.end_vertex == ids[0]
                        && he.start_vertex(&mesh) == Some(ids[1]))
                    .then_some(he_id)
                })
            })
            .expect("border edge 0-1 must exist");

        assert!(
            mesh.can_collapse_edge(border),
            "a border edge whose endpoints share only the opposite vertex is collapsible"
        );
    }
}

/// Captured collapse-phase weld failures (the `WELD_CAPTURE=1` extractor's
/// collapse driver, minimised from the two second-fixture soak failures):
/// regions of the weld mesh that `collapse_until_edges_above_min_length` opens
/// on replay, exactly as it did in the full weld.
///
/// These were **open cases**: the collapse left the region with boundary edges it
/// did not have. The test asserts the *fixed* behavior (watertight), so it failed
/// while the defect was real and flipped green once the collapse's own hole
/// closer landed. See [`test_weld_collapse_cases_stay_watertight`].
///
/// - `hole_gap125_seed19`: (n=8, gap=1.25, amount=0.625) under `WELD_ORDER_SEED=19`;
///   the replay gained 3 boundary edges on a previously-closed region.
/// - `wire_gap15`: (n=10, gap=1.5, amount=0.75) shipping order; the full weld left
///   a wire edge (both halves faceless), the replay gained 3 boundary edges
///   between interior vertices.
/// - `hole_gap01`: (n=9, gap=0.1, amount=0.05) shipping order; the replay gained 4
///   boundary edges forming a quad, which the triangle-only closer could not reach.
///
/// All three reproduce standalone at radius 1 with `min_len_sqr = 0.0096` (the
/// weld's sculpt-param threshold), which is what the replays use.
#[cfg(test)]
mod weld_collapse_cases {
    use super::edge_key;
    use crate::hash::HashSet;
    use crate::{MeshGraph, VertexId};
    use glam::Vec3;

    /// One captured collapse region: vertex positions, face triples, the
    /// outgoing-halfedge order per vertex whose star is inside the cut (see the
    /// merge cases — rebuilds derive a different order, and the ops read it),
    /// the protected vertices, and the collapse threshold.
    struct CollapseCase {
        what: &'static str,
        verts: &'static [(f32, f32, f32)],
        faces: &'static [(usize, usize, usize)],
        outgoing: &'static [(usize, &'static [usize], usize)],
        protected: &'static [usize],
        min_len_sqr: f32,
    }

    const HOLE_GAP125_SEED19: CollapseCase = CollapseCase {
        what: "hole (n=8, gap=1.25, amount=0.625) under WELD_ORDER_SEED=19",
        verts: &[
            (1.6497675, 0.6422053, 0.6137251),
            (1.6497675, 0.6422053, 0.6137251),
            (1.6500558, 0.6134274, 0.5847738),
            (1.652962, 0.6302906, 0.5859062),
            (1.6337953, 0.701779, 0.7528194),
            (1.6645867, 0.6977433, 0.590436),
        ],
        faces: &[
            (3, 0, 5),
            (0, 2, 1),
            (1, 4, 0),
            (5, 1, 3),
            (4, 5, 0),
            (3, 1, 2),
            (0, 3, 2),
            (4, 1, 5),
        ],
        outgoing: &[
            (0, &[1, 4, 3, 5, 2], 1),
            (1, &[3, 0, 5, 4, 2], 2),
            (2, &[0, 3, 1], 0),
            (3, &[0, 1, 2, 5], 0),
            (4, &[1, 0, 5], 1),
            (5, &[3, 4, 0, 1], 1),
        ],
        protected: &[2],
        min_len_sqr: 0.0096,
    };

    const WIRE_GAP15: CollapseCase = CollapseCase {
        what: "wire (n=10, gap=1.5, amount=0.75) shipping order",
        verts: &[
            (1.7879505, 0.71922123, 0.2361454),
            (1.7879505, 0.71922123, 0.2361454),
            (1.789509, 0.7494155, 0.24826732),
            (1.7879508, 0.71922123, 0.2361454),
            (1.7817173, 0.598444, 0.18765773),
        ],
        faces: &[
            (4, 0, 3),
            (1, 0, 4),
            (4, 3, 1),
            (1, 2, 0),
            (3, 2, 1),
            (3, 0, 2),
        ],
        outgoing: &[
            (0, &[3, 4, 1, 2], 2),
            (1, &[2, 0, 4, 3], 2),
            (2, &[3, 1, 0], 0),
            (3, &[0, 4, 1, 2], 0),
            (4, &[0, 3, 1], 0),
        ],
        protected: &[0, 3],
        min_len_sqr: 0.0096,
    };
    const HOLE_GAP01: CollapseCase = CollapseCase {
        what: "hole (n=9, gap=0.1, amount=0.05) shipping order",
        verts: &[
            (1.0483727, 0.24691357, -0.009315977),
            (1.05, 0.22222222, 0.11111111),
            (1.0613945, 0.32488927, 0.076331384),
            (1.05, 0.22222222, 0.22222222),
            (1.051393, 0.3640269, 0.21421906),
            (1.05, 0.33333334, 0.33333334),
            (1.051393, 0.3640269, 0.21421906),
            (1.0724406, 0.39210516, 0.089007884),
            (1.0921084, 0.22222224, 0.015908416),
            (1.0500001, 0.22222222, 0.11111111),
            (1.0500001, 0.22222222, 0.22222222),
            (1.0651144, 0.339556, 0.09228663),
            (1.0500001, 0.33333334, 0.33333334),
            (1.0427585, 0.48097044, 0.106952205),
            (1.055572, 0.4561075, 0.30132067),
        ],
        faces: &[
            (2, 11, 7),
            (11, 2, 8),
            (0, 8, 2),
            (2, 1, 0),
            (2, 4, 1),
            (4, 3, 1),
            (4, 13, 14),
            (4, 5, 3),
            (14, 5, 4),
            (9, 11, 8),
            (10, 6, 9),
            (6, 11, 9),
            (12, 6, 10),
            (7, 11, 13),
            (6, 13, 11),
            (12, 14, 6),
            (14, 13, 6),
            (13, 2, 7),
            (2, 13, 4),
        ],
        outgoing: &[
            (2, &[0, 1, 13, 7, 4, 8, 11], 11),
            (4, &[2, 1, 3, 14, 5, 13], 1),
            (6, &[10, 9, 12, 11, 14, 13], 13),
            (7, &[11, 2, 13], 11),
            (11, &[7, 9, 8, 6, 13, 2], 2),
            (13, &[2, 11, 14, 6, 7, 4], 14),
        ],
        protected: &[7],
        min_len_sqr: 0.0096,
    };

    const WIRE_GAP005: CollapseCase = CollapseCase {
        what: "wire (n=8, gap=0.05) shipping order",
        verts: &[
            (0.915625, 0.94054645, 0.025993811),
            (0.875, 0.875, -0.025),
            (0.9966701, 0.9038354, 0.017202832),
            (1.0176777, 0.75, -0.01767767),
            (1.0176777, 0.625, -0.01767767),
            (1.0311714, 0.9405465, 0.08529029),
            (1.0176777, 1.0176777, 0.25),
            (1.0176777, 1.0176777, 0.375),
            (1.025, 0.625, 0.125),
            (1.025, 0.75, 0.125),
            (1.025, 0.875, 0.125),
            (1.025, 0.75, 0.25),
            (1.025, 0.875, 0.25),
            (1.025, 0.875, 0.375),
            (1.1401674, 0.80129445, 0.004497938),
            (1.035748, 0.80129445, 0.02507852),
            (1.0323223, 0.625, -0.01767767),
            (1.0323223, 1.0176777, 0.125),
            (1.0323223, 1.0176777, 0.25),
            (1.175, 1.025, 0.25),
            (1.0323223, 1.0176777, 0.375),
            (1.175, 1.025, 0.375),
            (1.025, 0.625, 0.125),
            (1.025, 0.75, 0.125),
            (1.025, 0.75, 0.25),
            (1.0028476, 0.9106346, 0.09729202),
            (1.025, 0.875, 0.25),
            (1.025, 0.875, 0.375),
            (0.9618898, 0.9225592, 0.01677136),
            (0.98688984, 0.9296722, 0.040550973),
            (1.0491166, 0.9342806, 0.04071947),
            (1.015888, 0.94617033, 0.04193986),
        ],
        faces: &[
            (3, 9, 4),
            (9, 8, 4),
            (2, 10, 3),
            (10, 9, 3),
            (30, 10, 2),
            (9, 11, 8),
            (10, 12, 9),
            (12, 11, 9),
            (5, 6, 10),
            (6, 12, 10),
            (12, 13, 11),
            (6, 7, 12),
            (7, 13, 12),
            (0, 25, 28),
            (25, 0, 15),
            (31, 28, 25),
            (18, 19, 17),
            (20, 21, 18),
            (21, 19, 18),
            (22, 23, 16),
            (23, 15, 16),
            (24, 23, 22),
            (23, 25, 15),
            (28, 31, 2),
            (24, 26, 23),
            (26, 25, 23),
            (27, 26, 24),
            (31, 25, 17),
            (26, 18, 25),
            (18, 17, 25),
            (27, 20, 26),
            (20, 18, 26),
            (5, 14, 29),
            (2, 1, 28),
        ],
        outgoing: &[
            (9, &[3, 4, 8, 10, 11, 12], 11),
            (10, &[2, 3, 9, 5, 12, 6, 30], 12),
            (12, &[10, 9, 11, 6, 13, 7], 13),
            (18, &[17, 19, 20, 21, 26, 25], 26),
            (23, &[22, 16, 15, 24, 25, 26], 25),
            (25, &[23, 15, 26, 17, 18, 31, 28, 0], 0),
            (26, &[24, 23, 25, 27, 18, 20], 18),
            (28, &[0, 2, 1, 31, 25], 25),
        ],
        protected: &[],
        min_len_sqr: 0.0096,
    };

    const HOLE_GAP005: CollapseCase = CollapseCase {
        what: "hole (n=8, gap=0.05) shipping order",
        verts: &[
            (0.915625, 0.94054645, 0.025993811),
            (0.875, 0.875, -0.025),
            (0.9966701, 0.9038354, 0.017202832),
            (1.0176777, 0.75, -0.01767767),
            (1.0176777, 0.625, -0.01767767),
            (1.0311714, 0.9405465, 0.08529029),
            (1.0176777, 1.0176777, 0.25),
            (1.0176777, 1.0176777, 0.375),
            (1.025, 0.625, 0.125),
            (1.025, 0.75, 0.125),
            (1.025, 0.875, 0.125),
            (1.025, 0.75, 0.25),
            (1.025, 0.875, 0.25),
            (1.025, 0.875, 0.375),
            (1.0833333, 0.9649625, 0.03503754),
            (1.1401674, 0.80129445, 0.004497938),
            (1.035748, 0.80129445, 0.02507852),
            (1.0323223, 0.625, -0.01767767),
            (1.0323223, 1.0176777, 0.125),
            (1.0323223, 1.0176777, 0.25),
            (1.175, 1.025, 0.25),
            (1.0323223, 1.0176777, 0.375),
            (1.175, 1.025, 0.375),
            (1.025, 0.625, 0.125),
            (1.025, 0.75, 0.125),
            (1.025, 0.75, 0.25),
            (1.0028476, 0.9106346, 0.09729202),
            (1.025, 0.875, 0.25),
            (1.025, 0.875, 0.375),
            (0.9618898, 0.9225592, 0.01677136),
            (0.98688984, 0.9296722, 0.040550973),
            (1.0491166, 0.9342806, 0.04071947),
            (1.015888, 0.94617033, 0.04193986),
        ],
        faces: &[
            (2, 3, 1),
            (3, 9, 4),
            (9, 8, 4),
            (2, 10, 3),
            (10, 9, 3),
            (9, 11, 8),
            (10, 12, 9),
            (12, 11, 9),
            (6, 12, 10),
            (12, 13, 11),
            (6, 7, 12),
            (7, 13, 12),
            (0, 26, 29),
            (26, 0, 16),
            (32, 29, 26),
            (19, 20, 18),
            (21, 22, 19),
            (22, 20, 19),
            (23, 24, 17),
            (24, 16, 17),
            (25, 24, 23),
            (24, 26, 16),
            (25, 27, 24),
            (27, 26, 24),
            (28, 27, 25),
            (32, 26, 18),
            (31, 2, 14),
            (27, 19, 26),
            (19, 18, 26),
            (28, 21, 27),
            (21, 19, 27),
            (5, 15, 30),
            (5, 10, 31),
            (14, 2, 32),
            (18, 14, 32),
        ],
        outgoing: &[
            (2, &[29, 1, 3, 10, 31, 14, 32], 32),
            (9, &[3, 4, 8, 10, 11, 12], 11),
            (10, &[2, 3, 9, 5, 12, 6, 31], 12),
            (12, &[10, 9, 11, 6, 13, 7], 13),
            (19, &[18, 20, 21, 22, 27, 26], 27),
            (24, &[23, 17, 16, 25, 26, 27], 26),
            (26, &[24, 16, 27, 18, 19, 32, 29, 0], 0),
            (27, &[25, 24, 26, 28, 19, 21], 19),
            (29, &[0, 2, 1, 32, 26], 26),
            (32, &[26, 18, 14, 2, 29], 29),
        ],
        protected: &[],
        min_len_sqr: 0.0096,
    };

    /// Every captured case: all of them must finish watertight, on a mesh whose boundary
    /// is the cut's rim before and after, with no edge below the threshold lost.
    const CASES: &[CollapseCase] = &[HOLE_GAP125_SEED19, WIRE_GAP15, HOLE_GAP01, WIRE_GAP005];

    /// Boundary halfedges whose twin is face-less too: a wire edge contributes two.
    fn wire_halves(mesh: &MeshGraph) -> usize {
        mesh.halfedges
            .iter()
            .filter(|(_, he)| {
                he.face.is_none()
                    && he
                        .twin
                        .and_then(|t| mesh.halfedges.get(t))
                        .is_some_and(|t| t.face.is_none())
            })
            .count()
    }

    impl CollapseCase {
        fn build(&self) -> (MeshGraph, Vec<VertexId>) {
            self.build_into(MeshGraph::new())
        }

        /// [`Self::build`] into a given empty mesh (e.g. one from
        /// `utils::mesh_with_key_generation`).
        fn build_into(&self, mut mesh_graph: MeshGraph) -> (MeshGraph, Vec<VertexId>) {
            let mut vertex_ids = Vec::with_capacity(self.verts.len());
            for &(x, y, z) in self.verts {
                vertex_ids.push(mesh_graph.add_vertex(Vec3::new(x, y, z)));
            }
            for &(a, b, c) in self.faces {
                mesh_graph
                    .add_face_from_vertices(vertex_ids[a], vertex_ids[b], vertex_ids[c])
                    .expect("failed to rebuild a captured collapse region face");
            }

            for (idx, ends, seed) in self.outgoing {
                let v_id = vertex_ids[*idx];
                let Some(out_hes) = mesh_graph.outgoing_halfedges.get(v_id).cloned() else {
                    continue;
                };
                let mut reordered = Vec::with_capacity(out_hes.len());
                let mut left = out_hes;
                for end_idx in *ends {
                    let want = vertex_ids[*end_idx];
                    if let Some(pos) = left.iter().position(|he_id| {
                        mesh_graph
                            .halfedges
                            .get(*he_id)
                            .is_some_and(|he| he.end_vertex == want)
                    }) {
                        reordered.push(left.remove(pos));
                    }
                }
                reordered.extend(left);

                let seed_v = vertex_ids[*seed];
                let seed_he = reordered.iter().copied().find(|he_id| {
                    mesh_graph
                        .halfedges
                        .get(*he_id)
                        .is_some_and(|he| he.end_vertex == seed_v)
                });
                mesh_graph.outgoing_halfedges.insert(v_id, reordered);
                if let Some(seed_he) = seed_he
                    && let Some(vert) = mesh_graph.vertices.get_mut(v_id)
                {
                    vert.outgoing_halfedge = Some(seed_he);
                }
            }

            (mesh_graph, vertex_ids)
        }

        /// Read-only: every boundary halfedge the mesh has after the replay, with the
        /// captured index of both endpoints, whether the twin carries a face, and how
        /// many outgoing boundary halfedges its start vertex has (the number
        /// `sole_new_boundary_out` needs to be exactly one).
        fn diag_boundary(&self) {
            let (mut mesh_graph, vertex_ids) = self.build();
            let idx = |v: VertexId| {
                vertex_ids
                    .iter()
                    .position(|candidate| *candidate == v)
                    .map(|i| i.to_string())
                    .unwrap_or_else(|| format!("{v:?}"))
            };

            let mut marked = HashSet::default();
            for i in self.protected {
                marked.insert(vertex_ids[*i]);
            }

            let before = mesh_graph.boundary_edge_pairs();
            println!(
                "DIAG wires before {} boundary {} ",
                wire_halves(&mesh_graph),
                before.len()
            );
            mesh_graph.collapse_until_edges_above_min_length(self.min_len_sqr, &mut marked);
            println!(
                "DIAG wires after {} boundary {}",
                wire_halves(&mesh_graph),
                mesh_graph.boundary_edge_pairs().len()
            );

            for (he_id, he) in mesh_graph.halfedges.iter() {
                if he.face.is_some() {
                    continue;
                }

                let start = he.start_vertex(&mesh_graph);
                let twin_faced = he
                    .twin
                    .and_then(|t| mesh_graph.halfedges.get(t))
                    .is_some_and(|t| t.face.is_some());
                let out_boundary = start
                    .and_then(|v| mesh_graph.outgoing_halfedges.get(v))
                    .map(|hes| {
                        hes.iter()
                            .filter(|h| {
                                mesh_graph
                                    .halfedges
                                    .get(**h)
                                    .is_some_and(|x| x.face.is_none())
                            })
                            .count()
                    })
                    .unwrap_or(0);

                let new_edge = start.is_some_and(|v| !before.contains(&edge_key(v, he.end_vertex)));
                let out_new = start
                    .and_then(|v| mesh_graph.outgoing_halfedges.get(v))
                    .map(|hes| {
                        hes.iter()
                            .filter(|h| {
                                mesh_graph.halfedges.get(**h).is_some_and(|x| {
                                    x.face.is_none()
                                        && x.start_vertex(&mesh_graph).is_some_and(|sv| {
                                            !before.contains(&edge_key(sv, x.end_vertex))
                                        })
                                })
                            })
                            .count()
                    })
                    .unwrap_or(0);

                println!(
                    "DIAG he {he_id:?} {} -> {} twin_faced {twin_faced} new_edge {new_edge} \
                     out_new {out_new} out_boundary_at_start {out_boundary}",
                    start.map(idx).unwrap_or_default(),
                    idx(he.end_vertex)
                );
            }
        }

        /// Runs the collapse replay and reports the boundary edges gained
        /// between vertices that were interior to the cut (a cut's own rim is
        /// boundary in the replay but not in the weld, so it is ignored) plus
        /// the wire halves left behind.
        fn replay(&self) -> (usize, usize, Vec<String>) {
            self.replay_with(|mesh_graph, marked| {
                mesh_graph.collapse_until_edges_above_min_length(self.min_len_sqr, marked);
            })
        }

        /// [`Self::replay`] with the collapse call supplied by the caller, e.g. a scoped
        /// one. `run` gets the rebuilt mesh and the protected vertices.
        fn replay_with(
            &self,
            run: impl FnOnce(&mut MeshGraph, &mut HashSet<VertexId>),
        ) -> (usize, usize, Vec<String>) {
            let (mut mesh_graph, vertex_ids) = self.build();

            let boundary = |mesh: &MeshGraph| -> std::collections::BTreeSet<(VertexId, VertexId)> {
                mesh.halfedges
                    .iter()
                    .filter(|(_, he)| he.face.is_none())
                    .filter_map(|(_, he)| {
                        let start = he.start_vertex(mesh)?;
                        let end = he.end_vertex;
                        Some(if start < end {
                            (start, end)
                        } else {
                            (end, start)
                        })
                    })
                    .collect()
            };

            let before = boundary(&mesh_graph);
            let rim: std::collections::BTreeSet<VertexId> =
                before.iter().flat_map(|(a, b)| [*a, *b]).collect();
            let wires_before = wire_halves(&mesh_graph);

            let mut marked = HashSet::default();
            for idx in self.protected {
                marked.insert(vertex_ids[*idx]);
            }
            run(&mut mesh_graph, &mut marked);

            let after = boundary(&mesh_graph);
            let opened: Vec<String> = after
                .difference(&before)
                .filter(|(a, b)| !rim.contains(a) && !rim.contains(b))
                .map(|(a, b)| format!("{a:?}-{b:?}"))
                .collect();

            // Gained, not absolute: a cut can carry a wire on its own rim, which is the
            // capture's business and not the op's.
            let wire = wire_halves(&mesh_graph).saturating_sub(wires_before);

            (opened.len(), wire, opened)
        }
    }

    /// These captured regions reproduce the collapse-phase weld defect: the
    /// cleanup (`make_vertex_neighborhood_manifold` → `remove_neighboring_flaps`)
    /// drops the coincident face pair on a closed region and re-pairs the twins
    /// wrongly, leaving boundary edges (and, in the full weld, a wire). The
    /// test asserts the *fixed* behavior — the collapse must leave the region as
    /// watertight as it found it. `close_collapse_boundary_loops` closes the
    /// three- and four-halfedge loops the collapse opens, so this stays green
    /// only while that fix holds.
    #[test]
    fn test_weld_collapse_cases_stay_watertight() {
        let mut failures = Vec::new();
        for case in CASES {
            let (opened, wire, edges) = case.replay();
            if opened != 0 || wire != 0 {
                failures.push(format!(
                    "  {:?}: collapse gained {opened} boundary edges ({}: {}) and {wire} wire \
                     halves — the flap-removal defect is still open",
                    case.what,
                    edges.len(),
                    edges.join(", ")
                ));
            }
        }
        assert!(
            failures.is_empty(),
            "{} of {} collapse cases leave the region open:\n{}",
            failures.len(),
            CASES.len(),
            failures.join("\n"),
        );
    }

    /// KNOWN FAILURE - re-captured with `WELD_CAPTURE=1` after the wire and quad closers
    /// landed: the smallest cut of the gap=0.05 weld failure the collapse still opens. One
    /// boundary edge is gained, and it is the closer's own fan diagonal (27-32) left with a
    /// face-less half: the hole it closed continues into the cut's rim, which is boundary
    /// that predates the op, so the walk stops there and the rest of the hole stays.
    ///
    /// Whether that is the weld's problem or the cut's is open - the weld itself leaves 29
    /// (n=8) and 54 (n=9) boundary halfedges at this gap, none of them wires, and its n=9
    /// vertex-star split is gone.
    ///
    /// `#[ignore]`d while it fails - un-ignore it and add it to [`CASES`] with the fix.
    #[test]
    #[ignore = "currently fails: the gap=0.05 case is still open"]
    fn test_hole_gap_005_stays_watertight() {
        let (opened, wire, edges) = HOLE_GAP005.replay();
        assert!(
            opened == 0 && wire == 0,
            "{:?}: collapse gained {opened} boundary edges ({}: {}) and {wire} wire halves",
            HOLE_GAP005.what,
            edges.len(),
            edges.join(", ")
        );
    }

    /// Read-only: the shape of what the gap=0.05 case leaves open.
    #[test]
    #[ignore]
    fn diag_hole_gap_005() {
        HOLE_GAP005.diag_boundary();
    }

    /// The collapse, its hole closer and its wire removal must not depend on hash iteration
    /// order: every captured case, rebuilt under keys of different versions (same topology,
    /// different hashes), must end in the same mesh down to the slot indices.
    #[test]
    fn test_weld_collapse_cases_are_order_independent() {
        const GENERATIONS: u32 = 32;

        for case in CASES {
            let mut fingerprints = std::collections::BTreeSet::<String>::new();

            for generation in 0..GENERATIONS {
                let (mut mesh_graph, vertex_ids) =
                    case.build_into(crate::utils::mesh_with_key_generation(generation));
                let mut marked = HashSet::default();
                for idx in case.protected {
                    marked.insert(vertex_ids[*idx]);
                }
                let outcome =
                    mesh_graph.collapse_until_edges_above_min_length(case.min_len_sqr, &mut marked);

                let mut marked: Vec<VertexId> = marked.into_iter().collect();
                marked.sort_unstable();
                fingerprints.insert(crate::utils::strip_key_versions(&format!(
                    "{} | {outcome:?} | marked {marked:?}",
                    crate::utils::topology_fingerprint(&mesh_graph)
                )));
            }

            assert_eq!(
                fingerprints.len(),
                1,
                "{}: the same input under {GENERATIONS} key generations produced {} \
                 different results",
                case.what,
                fingerprints.len(),
            );
        }
    }

    /// The scoped collapse runs the same hole closer, and the holes it opens sit at the
    /// rim of the selection: only the faces next to a short edge are selected, so the
    /// faces the collapse removes and the closer replaces straddle the scope's border.
    /// Every captured case must still end watertight, with a selection whose reported
    /// delta replays exactly.
    #[test]
    fn test_scoped_collapse_closes_holes_at_the_rim_of_the_selection() {
        use crate::ops::selection_scoped_tests::{Keys, assert_bookkeeping};
        use crate::utils::element_keys;
        use crate::{FaceId, Selection};

        let mut closed_across_the_rim = 0;

        for case in CASES {
            let (opened, wire, edges) = case.replay_with(|mesh_graph, marked| {
                let mut faces: Vec<FaceId> = Vec::new();
                for (_, he) in &mesh_graph.halfedges {
                    if let Some(face) = he.face
                        && he.length_squared(mesh_graph) < case.min_len_sqr
                        && !faces.contains(&face)
                    {
                        faces.push(face);
                    }
                }
                assert!(!faces.is_empty(), "{}: nothing to collapse", case.what);

                let mut selection = Selection {
                    faces: faces.into_iter().collect(),
                    ..Default::default()
                };
                let sel_before = selection.clone();
                let before: Keys = element_keys(mesh_graph);

                let result = mesh_graph.collapse_selected_until_edges_above_min_length(
                    &mut selection,
                    case.min_len_sqr,
                    marked,
                );

                assert_bookkeeping(mesh_graph, &before, &sel_before, &selection, &result);

                // Count the closer's faces that share an edge with a face that was there
                // before and never selected: a hole closed across the rim.
                for &f_id in &result.added.faces {
                    for he_id in mesh_graph.faces[f_id].halfedges(mesh_graph) {
                        let neighbour = mesh_graph.halfedges[he_id]
                            .twin
                            .and_then(|twin_id| mesh_graph.halfedges[twin_id].face);
                        if let Some(neighbour) = neighbour
                            && before.2.contains(&neighbour)
                            && !sel_before.faces.contains(&neighbour)
                        {
                            closed_across_the_rim += 1;
                        }
                    }
                }
            });

            assert!(
                opened == 0 && wire == 0,
                "{:?}: scoped collapse gained {opened} boundary edges ({}) and {wire} wire halves",
                case.what,
                edges.join(", ")
            );
        }

        // Collapsing never builds faces; only the closer does. Without this the test
        // could pass on cases whose holes the scope kept from opening, or kept away
        // from the rim.
        assert!(
            closed_across_the_rim > 0,
            "no scoped collapse closed a hole next to an unselected face"
        );
    }

    /// Read-only: the replay's raw numbers, for the diag run.
    #[test]
    #[ignore]
    fn diag_collapse_cases() {
        for case in CASES {
            let (opened, wire, _) = case.replay();
            println!(
                "DIAG collapse case {:?}: +{opened} boundary edges, {wire} wire halves \
                 (replay threshold {})",
                case.what, case.min_len_sqr
            );
        }
    }
}

/// The closer's own shapes, on a mesh built for them: a pyramid whose base is an open
/// quad, the smallest mesh with a four-halfedge boundary loop.
#[cfg(test)]
mod boundary_loop_closer {
    use crate::hash::HashSet;
    use crate::{HalfedgeId, MeshGraph, VertexId};
    use glam::Vec3;

    /// A pyramid over the rim `v0(0,0,0) v1(3,0,0) v2(0.4,1.2,0) v3(0,0.3,0)`: five
    /// vertices, four faces, base left open as the quad hole. The rim is deliberately
    /// not a rectangle, so the two diagonals a fan could cut differ by a lot: `v0—v2`
    /// is 1.6 squared and `v1—v3` is 9.09.
    fn pyramid_with_quad_base() -> (MeshGraph, [VertexId; 4]) {
        let mut mesh = MeshGraph::new();
        let rim = [
            mesh.add_vertex(Vec3::new(0.0, 0.0, 0.0)),
            mesh.add_vertex(Vec3::new(3.0, 0.0, 0.0)),
            mesh.add_vertex(Vec3::new(0.4, 1.2, 0.0)),
            mesh.add_vertex(Vec3::new(0.0, 0.3, 0.0)),
        ];
        let apex = mesh.add_vertex(Vec3::new(1.5, 0.6, 1.0));

        // Wound so the rim halfedges stay face-less: each side face carries the
        // halfedge pointing away from the base.
        for i in 0..4 {
            mesh.add_face_from_vertices(rim[(i + 1) % 4], rim[i], apex)
                .expect("pyramid side face");
        }

        (mesh, rim)
    }

    /// The rim's halfedges in loop order: walk the single outgoing boundary halfedge
    /// of each vertex until the walk returns to `start`.
    fn rim_loop(mesh: &MeshGraph, start: VertexId) -> Vec<HalfedgeId> {
        let mut loop_hes = Vec::new();
        let mut he_id = mesh.sole_boundary_out(start).expect("the rim starts here");

        loop {
            loop_hes.push(he_id);

            let end_v = mesh.halfedges[he_id].end_vertex;
            if end_v == start {
                return loop_hes;
            }

            he_id = mesh.sole_boundary_out(end_v).expect("the rim continues");
        }
    }

    /// The vertices of the mesh's boundary, passed as the closer's candidates. The
    /// closer is handed them as if the op had opened that boundary - whether it then
    /// closes anything is what the guards decide.
    fn boundary_vertices(mesh: &MeshGraph) -> Vec<VertexId> {
        let mut vertices: Vec<VertexId> = Vec::new();

        for (a, b) in mesh.boundary_edge_pairs() {
            for v_id in [a, b] {
                if !vertices.contains(&v_id) {
                    vertices.push(v_id);
                }
            }
        }

        vertices
    }

    /// The cut diagonal must not follow the vertex the walk happened to start at: the
    /// walk starts from one of the closer's candidates, built from a hash set.
    #[test]
    fn test_quad_is_cut_along_the_same_diagonal_from_every_start() {
        let (mesh, rim) = pyramid_with_quad_base();
        let mut loop_hes = rim_loop(&mesh, rim[0]);

        let mut diagonals = HashSet::default();
        for _ in 0..loop_hes.len() {
            let corner = mesh.fan_corner(&loop_hes);
            let (he_a, he_b) = (loop_hes[corner], loop_hes[(corner + 1) % loop_hes.len()]);

            let start = mesh.halfedges[he_a]
                .start_vertex(&mesh)
                .expect("halfedge start");
            let end = mesh.halfedges[he_b].end_vertex;
            diagonals.insert((start.min(end), start.max(end)));

            loop_hes.rotate_left(1);
        }

        assert_eq!(
            diagonals.len(),
            1,
            "the diagonal changed with the start vertex: {diagonals:?}"
        );
        assert_eq!(
            diagonals.iter().next().copied(),
            Some((rim[0].min(rim[2]), rim[0].max(rim[2]))),
            "the fan runs from the loop's lowest-id vertex (v0), so it cuts v0—v2"
        );
    }

    /// The cut is a property of the loop: the length threshold is not consulted, because
    /// it decides only whether the cut edge has to be reported. (A cut below it is real —
    /// the weld's contact holes have one whose diagonal joins two coincident vertices —
    /// and the honest answer there is `Stalled`, not a different triangulation.)
    #[test]
    fn test_closer_reports_a_zero_length_diagonal() {
        let (mut mesh, rim) = pyramid_with_quad_base();
        // v2 onto v0: the canonical cut (v0—v2) is now a zero-length edge.
        mesh.positions[rim[2]] = mesh.positions[rim[0]];

        let candidates = boundary_vertices(&mesh);
        let none_was_boundary = HashSet::default();

        assert!(
            mesh.close_collapse_boundary_loops(&candidates, &none_was_boundary, 0.0096)
                .left_short_edge,
            "a zero-length edge the closer added must be reported, not hidden behind a \
             Converged"
        );
        assert!(
            mesh.boundary_edge_pairs().is_empty(),
            "and the hole must still close"
        );
    }

    /// A border that was already there must survive, and the same loop must close when
    /// it was not.
    #[test]
    fn test_closer_never_seals_a_pre_existing_border() {
        let (mut mesh, _) = pyramid_with_quad_base();
        let faces_before = mesh.faces.len();
        let boundary_before = mesh.boundary_edge_pairs();
        let candidates = boundary_vertices(&mesh);

        assert_eq!(
            boundary_before.len(),
            4,
            "the fixture's only boundary is the quad rim"
        );
        assert!(
            !mesh
                .close_collapse_boundary_loops(&candidates, &boundary_before, 0.0)
                .left_short_edge,
            "a refused loop adds no edge, so nothing can be short"
        );
        assert_eq!(
            mesh.faces.len(),
            faces_before,
            "the closer sealed a border that was meant to be open"
        );

        // The same loop, on a mesh where the boundary is new: it closes, in two faces -
        // the fan, then the triangle the fan leaves.
        let (mut mesh, _) = pyramid_with_quad_base();
        let none_was_boundary = HashSet::default();

        assert!(
            !mesh
                .close_collapse_boundary_loops(&candidates, &none_was_boundary, 0.0)
                .left_short_edge,
            "no added edge is below a threshold of 0"
        );
        assert_eq!(
            mesh.faces.len(),
            faces_before + 2,
            "a newly opened quad hole should close in two faces"
        );
        assert!(
            mesh.boundary_edge_pairs().is_empty(),
            "and leave no boundary behind"
        );
    }

    /// An edge the closer adds below the threshold is reported: the op may not call the
    /// mesh clean while it exists.
    #[test]
    fn test_closer_reports_an_added_edge_below_the_threshold() {
        let (mut mesh, _) = pyramid_with_quad_base();
        let candidates = boundary_vertices(&mesh);
        let none_was_boundary = HashSet::default();

        assert!(
            !mesh
                .close_collapse_boundary_loops(&candidates, &none_was_boundary, 0.0)
                .left_short_edge,
            "every edge in the fixture is longer than 0"
        );

        let (mut mesh, _) = pyramid_with_quad_base();

        assert!(
            mesh.close_collapse_boundary_loops(&candidates, &none_was_boundary, f32::MAX)
                .left_short_edge,
            "every added edge is below f32::MAX, so the closer must report one"
        );
    }

    /// A faced halfedge across the hole is the one case where the helper that adds the
    /// face would add a *second* edge between those two vertices, so the loop has to be
    /// left open.
    #[test]
    fn test_closer_refuses_a_diagonal_that_already_exists_faced() {
        let (mut mesh, rim) = pyramid_with_quad_base();
        let loop_hes = rim_loop(&mesh, rim[0]);
        let faces_before = mesh.faces.len();

        // `v2 -> v0` is exactly the diagonal the face over the loop's first two
        // halfedges would cut, and the extra face makes it an interior edge.
        let extra_apex = mesh.add_vertex(Vec3::new(1.5, 0.3, -0.5));
        mesh.add_face_from_vertices(rim[2], rim[0], extra_apex)
            .expect("the extra face");

        assert!(
            !mesh.can_close_loop_with(loop_hes[0], loop_hes[1]),
            "a faced diagonal must not be cut again - that would add a parallel edge"
        );
        assert!(
            mesh.can_close_loop_with(loop_hes[1], loop_hes[2]),
            "the other diagonal is free"
        );
        assert_eq!(
            mesh.faces.len(),
            faces_before + 1,
            "only the extra face was added"
        );
    }

    /// A collapse renames the vertex it absorbs, and with it every old border edge there.
    /// Driven through the real op: a hexagonal pyramid with an open base whose three short
    /// base edges collapse, which renames *every* remaining base edge. Without carrying the
    /// boundary snapshot through those renames, the closer takes the leftover triangle for
    /// a hole the op opened and seals a border that was open all along.
    #[test]
    fn test_collapse_keeps_an_old_border_whose_vertices_it_renamed() {
        let mut mesh = MeshGraph::new();
        let rim: Vec<VertexId> = [0.0_f32, 10.0, 120.0, 130.0, 240.0, 250.0]
            .iter()
            .map(|deg| {
                let rad = deg.to_radians();
                mesh.add_vertex(Vec3::new(rad.cos(), rad.sin(), 0.0))
            })
            .collect();
        let apex = mesh.add_vertex(Vec3::new(0.0, 0.0, 1.0));
        for i in 0..6 {
            mesh.add_face_from_vertices(rim[(i + 1) % 6], rim[i], apex)
                .expect("pyramid side face");
        }
        assert_eq!(mesh.boundary_edge_pairs().len(), 6);

        // The 10° base edges are ~0.03 squared; every other edge is above 1.
        mesh.collapse_until_edges_above_min_length(0.05, &mut HashSet::default());

        assert_eq!(
            mesh.faces.len(),
            3,
            "three collapses remove three faces and the closer must add none"
        );
        assert_eq!(
            mesh.boundary_edge_pairs().len(),
            3,
            "the base stays open, now a triangle"
        );
    }

    /// Which hole the closer closes first must follow the mesh, not the hash order of the
    /// candidate set: closing a quad allocates its diagonal, so the order decides which
    /// halfedge slots each diagonal gets. Two disjoint quad holes, rebuilt under keys of
    /// different versions, must come out identical down to the slot indices.
    #[test]
    fn test_closer_order_does_not_follow_hash_order() {
        const GENERATIONS: u32 = 32;

        let mut fingerprints = std::collections::BTreeSet::<String>::new();

        for generation in 0..GENERATIONS {
            let mut mesh = crate::utils::mesh_with_key_generation(generation);
            for offset in [0.0, 10.0] {
                let rim = [
                    mesh.add_vertex(Vec3::new(offset, 0.0, 0.0)),
                    mesh.add_vertex(Vec3::new(offset + 3.0, 0.0, 0.0)),
                    mesh.add_vertex(Vec3::new(offset + 0.4, 1.2, 0.0)),
                    mesh.add_vertex(Vec3::new(offset, 0.3, 0.0)),
                ];
                let apex = mesh.add_vertex(Vec3::new(offset + 1.5, 0.6, 1.0));
                for i in 0..4 {
                    mesh.add_face_from_vertices(rim[(i + 1) % 4], rim[i], apex)
                        .expect("pyramid side face");
                }
            }

            // As if the op had opened both bases.
            let none_was_boundary = HashSet::default();
            let candidates = mesh.opened_boundary_vertices(&none_was_boundary);
            mesh.close_collapse_boundary_loops(&candidates, &none_was_boundary, 0.0);

            assert!(mesh.boundary_edge_pairs().is_empty(), "both holes close");
            fingerprints.insert(crate::utils::topology_fingerprint(&mesh));
        }

        assert_eq!(
            fingerprints.len(),
            1,
            "the closer's result changed with the key generation"
        );
    }

    /// A three-halfedge hole around a lone triangle is emptied, not closed: closing it would
    /// build a zero-volume two-face pillow.
    #[test]
    fn test_closer_removes_a_lone_triangle_instead_of_building_a_pillow() {
        let mut mesh = MeshGraph::new();
        let a = mesh.add_vertex(Vec3::new(0.0, 0.0, 0.0));
        let b = mesh.add_vertex(Vec3::new(1.0, 0.0, 0.0));
        let c = mesh.add_vertex(Vec3::new(0.0, 1.0, 0.0));
        mesh.add_face_from_vertices(a, b, c).expect("the lone face");

        // As if the op had opened the whole boundary.
        let none_was_boundary = HashSet::default();
        let candidates = boundary_vertices(&mesh);
        let pass = mesh.close_collapse_boundary_loops(&candidates, &none_was_boundary, 0.0);

        assert!(pass.progressed);
        assert_eq!(mesh.faces.len(), 0, "the lone triangle is gone, no pillow");
        assert_eq!(
            pass.removed_halfedges.len(),
            6,
            "both halves of all three edges"
        );
        assert!(mesh.halfedges.is_empty());
        assert!(mesh.vertices.is_empty(), "its vertices had nothing else");
    }
}

#[cfg(test)]
mod tests {
    use glam::vec3;

    use crate::*;

    /// Removing a wire edge must leave live `outgoing_halfedge` seeds behind: the collapse
    /// runs `rebuild_outgoing_halfedges` before the wires go, so a seed that still names a
    /// wire halfedge would make the next `one_ring()` / `faces()` start from a dead id.
    #[test]
    fn test_remove_wire_edges_reseeds_endpoints() {
        let mut mesh = MeshGraph::new();
        let a = mesh.add_vertex(vec3(0.0, 0.0, 0.0));
        let b = mesh.add_vertex(vec3(1.0, 0.0, 0.0));
        let c = mesh.add_vertex(vec3(0.0, 1.0, 0.0));
        let tip = mesh.add_vertex(vec3(0.0, 2.0, 0.0));
        mesh.add_face_from_vertices(a, b, c).unwrap();

        // `add_edge` points both endpoints' seeds at the new (face-less) halves.
        let wire = mesh.add_edge(c, tip).unwrap();
        assert_eq!(
            mesh.vertices[c].outgoing_halfedge,
            Some(wire.start_to_end_he_id),
            "precondition: the seed of `c` names the wire"
        );

        let removed = mesh.remove_wire_edges(&[c, tip]);
        assert_eq!(removed.len(), 2, "one wire, both halves reported");
        assert!(!mesh.halfedges.contains_key(wire.start_to_end_he_id));
        assert!(!mesh.halfedges.contains_key(wire.twin_he_id));

        for v_id in [a, b, c] {
            if let Some(seed) = mesh.vertices[v_id].outgoing_halfedge {
                assert!(
                    mesh.halfedges.contains_key(seed),
                    "seed of {v_id:?} is dead after the wire removal"
                );
            }
        }
        assert!(
            !mesh.vertices.contains_key(tip),
            "`tip` was held only by the wire and must go with it"
        );
        assert!(
            mesh.vertices[c].outgoing_halfedge.is_some(),
            "`c` still has the triangle's halfedges and needs a seed"
        );

        let ring: Vec<_> = mesh.vertices[c].one_ring(&mesh).collect();
        assert!(!ring.is_empty());
        assert!(ring.iter().all(|he_id| mesh.halfedges.contains_key(*he_id)));
    }
}
