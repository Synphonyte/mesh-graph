use glam::Vec3;
use hashbrown::{HashMap, HashSet};
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    Face, FaceId, HalfedgeId, MeshGraph, VertexId, error_none,
    ops::{EdgeLengthCleanup, PendingEdges, PendingOrder},
    utils::unwrap_or_return,
};

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
        #[cfg(feature = "instrumentation")]
        crate::set_current_op("collapse");
        #[cfg(feature = "instrumentation")]
        crate::probe_chain_begin(self);
        let mut halfedges_to_collapse = PendingEdges::new(
            self.halfedges_map(|len_sqr| len_sqr < min_length_squared),
            PendingOrder::ShortestFirst,
        );

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
        let mut vertex_moved_at: HashMap<VertexId, u32> = HashMap::new();
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

            let collapse_edge_result = self.collapse_edge_inner(
                min_he_id,
                min_twin_id,
                min_start_v_id,
                min_end_v_id,
                min_center,
            );

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

            let mut halfedges_to_check = HashSet::new();

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

                    halfedges_to_check.insert(halfedge_id.min(twin_id));

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
        if halfedges_to_collapse.is_empty() {
            EdgeLengthCleanup::Converged
        } else {
            EdgeLengthCleanup::Stalled
        }
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

        self.check_inverted_faces(start_vertex_id, center)?;
        self.check_inverted_faces(end_vertex_id, center)?;

        Some((twin_id, start_vertex_id, end_vertex_id, center))
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

        let outcome = mg.collapse_until_edges_above_min_length(MIN_LEN_SQR, &mut HashSet::new());

        assert_eq!(outcome, EdgeLengthCleanup::Stalled);
        assert!(mesh_invariant_violations(&mg).is_empty());
    }

    #[test]
    fn test_collapse_reports_converged_when_it_drains() {
        let mut mg = build_grid(6);

        let outcome = mg.collapse_until_edges_above_min_length(2.0, &mut HashSet::new());

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

        let outcome = mg.collapse_until_edges_above_min_length(0.01, &mut HashSet::new());

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

        let mut marked = HashSet::new();
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

        mg.collapse_until_edges_above_min_length(2.0, &mut HashSet::new());

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
        mg.collapse_until_edges_above_min_length(3.0, &mut HashSet::new());

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

        mg.collapse_until_edges_above_min_length(MIN_LEN_SQR, &mut HashSet::new());

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

        assert!(marked.len() < before, "no vertex was collapsed - test is vacuous");
        for v_id in &marked {
            assert!(
                mg.vertices.contains_key(*v_id),
                "marked vertex {v_id:?} is dead"
            );
        }
    }
}
