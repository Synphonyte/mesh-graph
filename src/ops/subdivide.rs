use hashbrown::HashSet;
use tracing::instrument;

use crate::{
    HalfedgeId, MeshGraph, Selection, SelectionOps, VertexId, error_none,
    ops::{EdgeLengthCleanup, PendingEdges, PendingOrder},
    utils::unwrap_or_return,
};

impl MeshGraph {
    /// Subdivide all edges until all of them are <= max_length.
    /// Please note that you have to provide the squared value of max_length.
    ///
    /// Returns whether every edge ended up below the threshold, or the operation
    /// ran out of its work bound first — see [`EdgeLengthCleanup`].
    ///
    /// This will schedule necessary updates to the QBVH but you have to call
    /// `refit_bvh()` after the operation.
    #[instrument(skip(self))]
    pub fn subdivide_until_edges_below_max_length(
        &mut self,
        max_length_squared: f32,
        marked_halfedge_ids: &mut HashSet<HalfedgeId>,
        marked_vertex_ids: &mut HashSet<VertexId>,
    ) -> EdgeLengthCleanup {
        #[cfg(feature = "instrumentation")]
        crate::set_current_op("subdivide");
        #[cfg(feature = "instrumentation")]
        crate::probe_chain_begin(self);
        let pending = self.halfedges_map(|len_sqr| len_sqr > max_length_squared);

        // Bound the work by the initial problem size, not the mesh size: in stretched
        // regions (e.g. the punch band) splitting the longest edge of a triangle can
        // re-create a fan edge above the threshold, so the set can fail to drain and the
        // old `halfedges.len()` bound let the op burn the whole halfedge count per call
        // while growing the mesh ~6x per iteration. Healthy meshes drain at ~1.2x the
        // initial set size, so 2x leaves ample headroom and caps pathological blowups.
        //
        // The additive term scales with how *deep* the refinement has to go, because an
        // edge `r` times too long becomes `r` pieces and each split also splits the two
        // adjacent faces - so the work per over-long edge grows with `r`, not with the
        // number of over-long edges. A flat constant ignored that and made the bound bite
        // on small meshes needing deep refinement: a 6-edge tetrahedron taken to a
        // target of 0.2 needs ~200 splits but got a budget of 112, stopping less than
        // halfway through. On production meshes `r` is barely above 1, so this is within
        // a few hundred of the old value and the blowup guard is unchanged.
        let depth = if max_length_squared > 0.0 {
            let longest = pending.values().copied().fold(0.0_f32, f32::max);
            if longest.is_finite() {
                ((longest / max_length_squared).sqrt().ceil() as usize).clamp(1, 64)
            } else {
                1
            }
        } else {
            1
        };

        let budget = pending.len() * 2 + 100 * depth;

        let mut halfedges_to_subdivide = PendingEdges::new(pending, PendingOrder::LongestFirst);

        for _ in 0..budget {
            if halfedges_to_subdivide.is_empty() {
                break;
            }

            // #[cfg(feature = "rerun")]
            // self.log_hes_rerun(
            //     "subdivide/selection",
            //     &halfedges_to_subdivide
            //         .iter()
            //         .map(|(he, _)| *he)
            //         .collect::<Vec<_>>(),
            // );

            let Some((max_he_id, _)) = halfedges_to_subdivide.pop_live() else {
                // `PendingEdges::insert` pushes whenever it changes the map, so
                // running out of live heap entries means the set really is empty.
                debug_assert!(
                    halfedges_to_subdivide.is_empty(),
                    "heap drained but {} pending edges remain - a push site is missing",
                    halfedges_to_subdivide.len()
                );
                break;
            };

            halfedges_to_subdivide.remove(&max_he_id);

            let mut affected_faces = Selection::default();

            // already checked
            let max_he = self.halfedges[max_he_id];
            if let Some(face_id) = max_he.face {
                affected_faces.insert(face_id);
            }

            if let Some(twin_id) = max_he.twin.or_else(error_none!("Twin missing"))
                && let Some(twin_he) = self
                    .halfedges
                    .get(twin_id)
                    .or_else(error_none!("Halfedge not found"))
                && let Some(twin_face_id) = twin_he.face
            {
                affected_faces.insert(twin_face_id);
            }

            let subdivide_edge_result = unwrap_or_return!(
                self.subdivide_edge(max_he_id),
                "Couldn't subdivide edge",
                EdgeLengthCleanup::Stalled
            );

            if marked_halfedge_ids.contains(&max_he_id) {
                marked_halfedge_ids.extend(subdivide_edge_result.added_halfedges.iter().copied());
                marked_vertex_ids.insert(subdivide_edge_result.added_vertex);
            }

            // #[cfg(feature = "rerun")]
            // {
            //     crate::RR
            //         .log("meshgraph/subdivide", &rerun::Clear::recursive())
            //         .unwrap();
            //     crate::RR
            //         .log("meshgraph/halfedge/subdivide", &rerun::Clear::recursive())
            //         .unwrap();
            //     crate::RR
            //         .log("meshgraph/face/subdivide", &rerun::Clear::recursive())
            //         .unwrap();
            //     self.log_rerun();
            // }

            for new_he_id in subdivide_edge_result.added_halfedges {
                // newly inserted in `self.subdivide_edge`
                let new_he = self.halfedges[new_he_id];

                if let Some(face_id) = new_he.face {
                    affected_faces.insert(face_id);
                }

                let new_twin_id =
                    unwrap_or_return!(new_he.twin, "New twin missing", EdgeLengthCleanup::Stalled);
                let new_twin = unwrap_or_return!(
                    self.halfedges.get(new_twin_id),
                    "New twin not found",
                    EdgeLengthCleanup::Stalled
                );

                if let Some(face_id) = new_twin.face {
                    affected_faces.insert(face_id);
                    #[cfg(feature = "rerun")]
                    self.log_face_rerun("subdivide/selected_new", face_id);
                }
            }

            let mut new_hes_to_check = HashSet::new();

            for he_id in affected_faces.resolve_to_halfedges(self) {
                let he = unwrap_or_return!(
                    self.halfedges.get(he_id),
                    "Halfedge not found",
                    EdgeLengthCleanup::Stalled
                );
                let twin_id =
                    unwrap_or_return!(he.twin, "Twin missing", EdgeLengthCleanup::Stalled);

                new_hes_to_check.insert(he_id.min(twin_id));
            }

            // Only ever inserts, never removes, which relies on `subdivide_edge` moving
            // no existing vertex (pinned by
            // `test_subdivide_adds_geometry_and_moves_no_existing_vertex`): every edge
            // here either kept both endpoints and so kept its length, or is new. The one
            // edge that does get shorter is the subdivided one, and it was dropped from
            // the pending set above before the split. Were vertices ever moved, an edge
            // that fell below the threshold would have to be removed here or it would
            // sit pending forever and eventually be split again below `max_length`.
            for he_id in new_hes_to_check {
                let he = self.halfedges[he_id]; // checked above

                let len_sqr = he.length_squared(self);

                if len_sqr > max_length_squared {
                    #[cfg(feature = "rerun")]
                    self.log_he_rerun("/subdivide/new_edge", he_id);

                    halfedges_to_subdivide.insert(he_id, len_sqr);
                }
            }
        }

        // Subdivision itself only adds geometry, but a marked set is long-lived and may
        // still carry ids invalidated by an earlier operation. Both ops hand back only
        // live keys.
        marked_vertex_ids.retain(|v_id| self.vertices.contains_key(*v_id));
        marked_halfedge_ids.retain(|he_id| self.halfedges.contains_key(*he_id));

        #[cfg(feature = "instrumentation")]
        if self.probe_chain_integrity("subdivide_until_edges_below_max_length") {
            crate::state_history_push(self, "subdivide_until_edges_below_max_length");
        }

        #[cfg(feature = "rerun")]
        self.log_rerun();

        // The loop only exits early once the pending set drains, so anything left in
        // it means the budget ran out with work outstanding.
        if halfedges_to_subdivide.is_empty() {
            EdgeLengthCleanup::Converged
        } else {
            EdgeLengthCleanup::Stalled
        }
    }

    /// Subdivides an edge by computing it's center vertex. This also subdivides any adjacent triangles and
    /// makes sure everything is properly reconnected. Works only on triangle meshes.
    ///
    /// Returns the id of the new halfedge which goes from the center vertex to the original edge's end vertex.
    /// And also return the halfedges that are created by subdividing the adjacent faces. Only one of the two twin
    /// halfedges per face subdivision is returned. In total the number `n` of halfedges returned is `1 <= n <= 3`.
    /// (The one from dividing the halfedge and at most 2 from dividing the two adjacent faces).
    ///
    /// Also returns the created vertex id.
    #[instrument(skip(self))]
    pub fn subdivide_edge(&mut self, halfedge_id: HalfedgeId) -> Option<SubdivideEdge> {
        let mut added_halfedges = Vec::with_capacity(3);

        let he = self
            .halfedges
            .get(halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;
        let twin_id = he.twin.or_else(error_none!("Twin halfedge not found"))?;

        let start_v = he
            .start_vertex(self)
            .or_else(error_none!("Start vertex not found"))?;
        let end_v = he.end_vertex;

        let start_pos = self
            .positions
            .get(start_v)
            .or_else(error_none!("Start position not found"))?;
        let end_pos = self
            .positions
            .get(end_v)
            .or_else(error_none!("End position not found"))?;

        let center_pos = (start_pos + end_pos) * 0.5;

        // #[cfg(feature = "rerun")]
        // {
        //     crate::RR
        //         .log(
        //             "meshgraph/subdivide/edge",
        //             &rerun::Arrows3D::from_vectors([vec3_array(end_pos - start_pos)])
        //                 .with_origins([vec3_array(start_pos)]),
        //         )
        //         .unwrap();

        //     crate::RR
        //         .log(
        //             "meshgraph/subdivide/center",
        //             &rerun::Points3D::new([vec3_array(center_pos)]),
        //         )
        //         .unwrap();
        // }

        let center_v = self.add_vertex(center_pos);
        if let Some(normals) = &mut self.vertex_normals {
            let start_normal = normals
                .get(start_v)
                .or_else(error_none!("Start normal not found"))?;
            let end_normal = normals
                .get(end_v)
                .or_else(error_none!("End normal not found"))?;
            normals.insert(center_v, (start_normal + end_normal).normalize());
        }

        let new_he = self.add_halfedge(center_v, end_v)?;
        // inserted just above
        self.vertices[center_v].outgoing_halfedge = Some(new_he);

        added_halfedges.push(new_he);

        if let Some(new_face_he) = self.subdivide_face(halfedge_id, new_he, center_v) {
            added_halfedges.push(new_face_he);
        } else {
            // The face side of the subdivided edge is boundary: `subdivide_face` early-returns
            // without re-pointing `halfedge_id` at the center vertex. Do it here so the twin
            // re-pairing below yields a consistent pair (the boundary halfedge of the
            // subdivided edge) instead of pairing `new_he` with a halfedge that still ends at
            // the old start vertex (which misattributes `new_he` and breaks the boundary).
            self.halfedges[halfedge_id].end_vertex = center_v;
        }

        let new_twin = self.add_halfedge(center_v, start_v)?;

        if let Some(new_face_he) = self.subdivide_face(twin_id, new_twin, center_v) {
            added_halfedges.push(new_face_he);
        } else {
            // Same as above for the twin side (the twin of the subdivided edge is a boundary
            // halfedge without a face).
            self.halfedges[twin_id].end_vertex = center_v;
        }

        // inserted above
        self.halfedges[new_he].twin = Some(twin_id);
        self.halfedges
            .get_mut(twin_id)
            .or_else(error_none!("Twin halfedge not found"))?
            .twin = Some(new_he);

        // checked in the beginning of the function
        self.halfedges[halfedge_id].twin = Some(new_twin);
        // inserted above
        self.halfedges[new_twin].twin = Some(halfedge_id);

        // self.vertices[end_v].outgoing_halfedge = Some(new_twin);
        // self.vertices[start_v].outgoing_halfedge = Some(new_he);

        Some(SubdivideEdge {
            added_halfedges,
            added_vertex: center_v,
        })
    }

    /// Subdivides a triangle into two halves. Used in [Self::subdivide_edge].
    #[instrument(skip(self))]
    fn subdivide_face(
        &mut self,
        existing_halfedge_id: HalfedgeId,
        new_halfedge_id: HalfedgeId,
        center_v: VertexId,
    ) -> Option<HalfedgeId> {
        let he = self
            .halfedges
            .get(existing_halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;

        let face_id = he.face?;
        self.faces
            .get_mut(face_id)
            .or_else(error_none!("Facee not found"))?
            .halfedge = existing_halfedge_id;

        // checked above
        let next_he = self.halfedges[existing_halfedge_id]
            .next
            .or_else(error_none!("Next halfedge missing"))?;
        let last_he = self
            .halfedges
            .get(next_he)
            .or_else(error_none!("Next halfedge not found"))?
            .next
            .or_else(error_none!("Last halfedge not found"))?;

        // Validate everything that can fail *before* mutating. The re-wiring below
        // splices a new halfedge into the face chain and creates `new_he`;
        // aborting in between (e.g. `add_halfedge(next_he.end, ...)` needs
        // `next_he.end` to be a live vertex) would leave the chain spliced with an
        // unpaired halfedge and the old `next_he` orphaned — the layer-4 corruption.
        if !self
            .vertices
            .contains_key(self.halfedges[next_he].end_vertex)
        {
            tracing::error!("subdivide_face: next halfedge {next_he:?} ends at a missing vertex");
            return None;
        }
        if !self.vertices.contains_key(center_v) {
            tracing::error!("subdivide_face: center vertex {center_v:?} is missing");
            return None;
        }

        // rewire existing face
        let new_he = self.add_halfedge(center_v, self.halfedges[next_he].end_vertex)?; // checked above

        self.halfedges[existing_halfedge_id].next = Some(new_he); // checked above
        self.halfedges[new_he].next = Some(last_he); // inserted above
        self.halfedges[new_he].face = Some(face_id); // inserted above

        let new_twin = self.add_halfedge(self.halfedges[next_he].end_vertex, center_v)?; // checked above

        // insert new face
        let new_face_id = self.add_face(new_halfedge_id, next_he, new_twin);

        self.halfedges[new_twin].twin = Some(new_he); // inserted above
        self.halfedges[new_he].twin = Some(new_twin); // inserted above

        self.halfedges[existing_halfedge_id].end_vertex = center_v; // checked above

        let face = self.faces[face_id]; // checked above
        let new_face = self.faces[new_face_id]; // inserted above
        self.bvh
            .insert_or_update_partially(face.aabb(self), face.index, 0.0);
        self.bvh
            .insert_or_update_partially(new_face.aabb(self), new_face.index, 0.0);

        // #[cfg(feature = "rerun")]
        // {
        //     self.log_he_rerun("subdivide/new_he", new_he);
        //     self.log_he_rerun("subdivide/new_twin", new_twin);
        // }

        Some(new_he)
    }
}

pub struct SubdivideEdge {
    /// All halfedges created by the subdivision.
    added_halfedges: Vec<HalfedgeId>,
    /// This is the center vertex of the subdivided edge that was created.
    added_vertex: VertexId,
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::ops::EdgeLengthCleanup;
    use crate::utils::{build_grid, mesh_invariant_violations};
    use glam::Vec3;
    use hashbrown::HashSet;

    /// `build_grid` uses unit cells, so canonical edge lengths squared are
    /// exactly 1.0 (axis-aligned) or 2.0 (diagonal).
    fn max_len_sqr(mg: &MeshGraph) -> f32 {
        mg.halfedges
            .values()
            .map(|he| he.length_squared(mg))
            .fold(0.0, f32::max)
    }

    fn count_above(mg: &MeshGraph, max_length_squared: f32) -> usize {
        mg.halfedges
            .values()
            .filter(|he| he.length_squared(mg) > max_length_squared)
            .count()
    }

    fn subdivide(mg: &mut MeshGraph, max_length_squared: f32) {
        mg.subdivide_until_edges_below_max_length(
            max_length_squared,
            &mut HashSet::new(),
            &mut HashSet::new(),
        );
    }

    /// The small-mesh case the old flat `+ 100` work bound could not reach.
    ///
    /// A unit tetrahedron taken to a target edge of 0.2 needs ~200 splits from only 6
    /// over-long edges. Under the old bound of `2 * 6 + 100` it stopped at 112 splits
    /// with 60 edges still over-long, so callers had to call again in a loop.
    #[test]
    fn test_subdivide_converges_on_a_small_mesh_needing_deep_refinement() {
        use glam::Vec3;

        let positions = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let indices: Vec<usize> = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
        let mut mg = MeshGraph::indexed_triangles(&positions, &indices);

        let target: f32 = 0.2;
        let outcome = mg.subdivide_until_edges_below_max_length(
            target * target,
            &mut HashSet::new(),
            &mut HashSet::new(),
        );

        assert_eq!(
            outcome,
            EdgeLengthCleanup::Converged,
            "one call left {} edges over-long",
            count_above(&mg, target * target)
        );
        assert_eq!(count_above(&mg, target * target), 0);
        assert!(mesh_invariant_violations(&mg).is_empty());
    }

    #[test]
    fn test_subdivide_reports_converged_when_it_drains() {
        let mut mg = build_grid(4);

        let outcome = mg.subdivide_until_edges_below_max_length(
            0.9,
            &mut HashSet::new(),
            &mut HashSet::new(),
        );

        assert_eq!(outcome, EdgeLengthCleanup::Converged);
        assert_eq!(count_above(&mg, 0.9), 0);
    }

    /// An already-clean mesh converges without doing anything.
    #[test]
    fn test_subdivide_reports_converged_on_a_clean_mesh() {
        let mut mg = build_grid(3);
        let faces = mg.faces.len();

        // Every edge is already below this.
        let outcome = mg.subdivide_until_edges_below_max_length(
            100.0,
            &mut HashSet::new(),
            &mut HashSet::new(),
        );

        assert_eq!(outcome, EdgeLengthCleanup::Converged);
        assert_eq!(mg.faces.len(), faces);
    }

    #[test]
    fn test_subdivide_until_max_length_holds_invariants() {
        let mut mg = build_grid(6);
        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "fixture is already broken"
        );

        let before_above = count_above(&mg, 0.5);
        let before_max = max_len_sqr(&mg);

        subdivide(&mut mg, 0.5);

        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "invariants violated: {:?}",
            mesh_invariant_violations(&mg)
        );
        // Deliberately weaker than "all edges below the threshold": the budget can
        // stop the op with work remaining, and that behaviour is unchanged.
        assert!(count_above(&mg, 0.5) < before_above);
        assert!(max_len_sqr(&mg) < before_max);
    }

    /// The postcondition the doc comment actually promises. A small grid and a
    /// threshold it can reach within budget, so a `pop_live` that silently drains
    /// the heap early fails here instead of passing quietly.
    #[test]
    fn test_subdivide_until_max_length_drains_completely() {
        let mut mg = build_grid(4);

        subdivide(&mut mg, 0.9);

        assert!(mesh_invariant_violations(&mg).is_empty());
        assert_eq!(
            count_above(&mg, 0.9),
            0,
            "subdivision left {} edges above the threshold",
            count_above(&mg, 0.9)
        );
    }

    /// Subdivision must only ever add geometry, never move an existing vertex.
    /// The insert-only re-check pass in `subdivide_until_edges_below_max_length`
    /// is unsound without this, so it is pinned rather than assumed.
    #[test]
    fn test_subdivide_adds_geometry_and_moves_no_existing_vertex() {
        let mut mg = build_grid(4);
        let before: Vec<(crate::VertexId, Vec3)> =
            mg.positions.iter().map(|(v, &p)| (v, p)).collect();
        let faces_before = mg.faces.len();

        subdivide(&mut mg, 0.9);

        assert!(mg.faces.len() > faces_before);
        for (v_id, pos) in before {
            assert_eq!(
                mg.positions.get(v_id),
                Some(&pos),
                "vertex {v_id:?} moved during subdivision"
            );
        }
    }

    #[test]
    fn test_subdivide_is_idempotent_once_converged() {
        let mut mg = build_grid(4);
        subdivide(&mut mg, 0.9);

        let verts = mg.vertices.len();
        let faces = mg.faces.len();
        let halfedges = mg.halfedges.len();

        subdivide(&mut mg, 0.9);

        assert_eq!(mg.vertices.len(), verts);
        assert_eq!(mg.faces.len(), faces);
        assert_eq!(mg.halfedges.len(), halfedges);
    }

    /// Pins that the *longest* edge is the one chosen, not merely some edge above
    /// the threshold. The grid's diagonals (len_sqr 2.0) are strictly longer than
    /// its axis edges (1.0), so a threshold between them admits only diagonals;
    /// every split must therefore land on a diagonal midpoint, which has exactly
    /// one half-integer coordinate pair.
    #[test]
    fn test_subdivide_picks_longest_edges_first() {
        let mut mg = build_grid(3);
        let before: HashSet<crate::VertexId> = mg.positions.keys().collect();

        // 1.0 < threshold < 2.0: only the diagonals qualify.
        subdivide(&mut mg, 1.5);

        let added: Vec<Vec3> = mg
            .positions
            .iter()
            .filter(|(v, _)| !before.contains(v))
            .map(|(_, &p)| p)
            .collect();

        assert!(!added.is_empty(), "expected at least one diagonal split");
        for p in added {
            let frac_x = (p.x - p.x.floor() - 0.5).abs();
            let frac_y = (p.y - p.y.floor() - 0.5).abs();
            assert!(
                frac_x < 1e-6 && frac_y < 1e-6,
                "split at {p:?} is not a diagonal midpoint, so a shorter edge was chosen"
            );
        }
    }

    /// Subdivision only adds geometry, so it cannot invalidate its own marked ids -
    /// but a marked set is long-lived and can arrive already carrying ones an earlier
    /// operation killed. Both ops hand back only live keys, so those have to be
    /// pruned here too.
    #[test]
    fn test_subdivide_purges_dead_ids_from_marked_sets() {
        let mut mg = build_grid(4);

        // Kill some ids first, so the marked sets carry stale entries into the call.
        let mut marked_vertices: HashSet<crate::VertexId> = mg.vertices.keys().collect();
        let mut marked_halfedges: HashSet<HalfedgeId> = mg.halfedges.keys().collect();
        mg.collapse_until_edges_above_min_length(1.5, &mut HashSet::new());
        assert!(
            marked_vertices
                .iter()
                .any(|v| !mg.vertices.contains_key(*v)),
            "fixture failed to invalidate anything - test is vacuous"
        );

        mg.subdivide_until_edges_below_max_length(0.5, &mut marked_halfedges, &mut marked_vertices);

        for v_id in &marked_vertices {
            assert!(
                mg.vertices.contains_key(*v_id),
                "marked vertex {v_id:?} is dead"
            );
        }
        for he_id in &marked_halfedges {
            assert!(
                mg.halfedges.contains_key(*he_id),
                "marked halfedge {he_id:?} is dead"
            );
        }
    }
}
