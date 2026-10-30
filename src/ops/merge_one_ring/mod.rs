mod rim;
mod rim_planner;
#[cfg(test)]
mod rim_probe;
#[cfg(test)]
mod tests;

use std::hash::Hash;
use std::ops::RangeInclusive;

use crate::hash::{HashMap, HashSet};
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    FaceId, HalfedgeId, MeshGraph, VertexId, error_none,
    ops::{
        add::AddFace,
        collapse::{CollapseEdge, VertexRenames, vertices_of_pairs},
        edit::MergeVertices,
    },
    utils::unwrap_or_return,
};

#[derive(Default)]
pub struct MergeVerticesOneRing {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    pub added_vertices: Vec<VertexId>,
    pub added_halfedges: Vec<HalfedgeId>,
    pub added_faces: Vec<FaceId>,
}

/// The three post-hoc repair passes of the merge ("the nets"):
/// `weld_parallel_edges` (before and after the neighborhood cleanup) and
/// `close_merge_opened_holes`.
///
/// Always on outside tests. Test builds can turn them off with
/// `MESH_GRAPH_MERGE_NETS=0`: with the nets off, the tests measure whether the band
/// itself closes exactly what the removals opened, instead of whether the nets mop
/// up afterwards.
///
/// With the nets off, the one failure is `weld_cases_stay_watertight`'s
/// `gap=0.05` case: its coincident fusion opens boundary before any band is
/// planned, and only the closer seals that.
#[cfg(test)]
fn merge_nets_enabled() -> bool {
    use std::sync::OnceLock;

    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var("MESH_GRAPH_MERGE_NETS").map_or(true, |value| value != "0" && value != "off")
    })
}

#[cfg(not(test))]
#[inline(always)]
fn merge_nets_enabled() -> bool {
    true
}

// How many coincident cross-ring pairs `merge_vertices_one_rings` fused on this thread.
// Tells whether contacts still reach the merge now that `zip_coincident_patches` runs
// first; if this stays 0, the fusion block can go.
#[cfg(any(test, feature = "instrumentation"))]
thread_local! {
    static COINCIDENT_FUSIONS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// The number of coincident cross-ring pairs `merge_vertices_one_rings` has fused on this
/// thread so far.
#[cfg(any(test, feature = "instrumentation"))]
pub fn coincident_fusion_count() -> usize {
    COINCIDENT_FUSIONS.with(|count| count.get())
}

/// Records that `id` was removed: an element this op added is simply no longer added, one
/// that was there before is removed.
fn record_removal<T: PartialEq>(added: &mut Vec<T>, removed: &mut Vec<T>, id: T) {
    if added.contains(&id) {
        added.retain(|added_id| *added_id != id);
    } else {
        removed.push(id);
    }
}

/// [`record_removal`] for a vertex or halfedge, which also leaves the caller's marked set.
///
/// [`MergeRecord::finish`] drops everything in `removed_*` from the marked sets, but an id
/// this op added and removed again is in neither list by then, while the op may already
/// have marked it.
fn record_marked_removal<T: Copy + Eq + Hash>(
    added: &mut Vec<T>,
    removed: &mut Vec<T>,
    marked: &mut HashSet<T>,
    id: T,
) {
    record_removal(added, removed, id);
    marked.remove(&id);
}

/// What the op has done so far, together with the caller's marked sets, which every
/// removal has to keep in step.
struct MergeRecord<'a> {
    result: MergeVerticesOneRing,
    marked_halfedges: &'a mut HashSet<HalfedgeId>,
    marked_vertices: &'a mut HashSet<VertexId>,
}

impl<'a> MergeRecord<'a> {
    fn new(
        marked_halfedges: &'a mut HashSet<HalfedgeId>,
        marked_vertices: &'a mut HashSet<VertexId>,
    ) -> Self {
        Self {
            result: MergeVerticesOneRing::default(),
            marked_halfedges,
            marked_vertices,
        }
    }

    fn vertex_removed(&mut self, v_id: VertexId) {
        record_marked_removal(
            &mut self.result.added_vertices,
            &mut self.result.removed_vertices,
            self.marked_vertices,
            v_id,
        );
    }

    fn halfedge_removed(&mut self, he_id: HalfedgeId) {
        record_marked_removal(
            &mut self.result.added_halfedges,
            &mut self.result.removed_halfedges,
            self.marked_halfedges,
            he_id,
        );
    }

    fn face_removed(&mut self, face_id: FaceId) {
        record_removal(
            &mut self.result.added_faces,
            &mut self.result.removed_faces,
            face_id,
        );
    }

    /// `face_id` was removed and took the vertices and halfedges `remove_face` returned
    /// with it.
    fn face_removed_with(
        &mut self,
        face_id: FaceId,
        (v_ids, he_ids): (Vec<VertexId>, Vec<HalfedgeId>),
    ) {
        self.face_removed(face_id);
        for he_id in he_ids {
            self.halfedge_removed(he_id);
        }
        for v_id in v_ids {
            self.vertex_removed(v_id);
        }
    }

    /// The result, with the removed ids dropped from the marked sets.
    ///
    /// The marked sets are long-lived (a caller keeps them across hundreds of merges), so
    /// they are pruned by the ids the op reports rather than scanned for dead ones.
    fn finish(self) -> MergeVerticesOneRing {
        for he_id in &self.result.removed_halfedges {
            self.marked_halfedges.remove(he_id);
        }
        for v_id in &self.result.removed_vertices {
            self.marked_vertices.remove(v_id);
        }
        self.result
    }
}

/// The vertices of both rings and the ones the op added, where its repair passes look.
fn ring_region(
    one_ring_v_ids1: &[VertexId],
    one_ring_v_ids2: &[VertexId],
    added_vertices: &[VertexId],
) -> Vec<VertexId> {
    one_ring_v_ids1
        .iter()
        .chain(one_ring_v_ids2)
        .chain(added_vertices)
        .copied()
        .collect()
}

impl MeshGraph {
    /// Merge two vertices by connecting their 1-rings.
    ///
    /// The vertices are deleted on top of everything that is returned in the `removed_...` fields.
    ///
    /// See [Freestyle: Sculpting meshes with self-adaptive topology DOI 10.1016/j.cag.2011.03.033](https://inria.hal.science/inria-00606516v1/document)
    /// Chapters 3.2 and 5.1
    #[instrument(skip(self, marked_halfedges, marked_vertices))]
    pub fn merge_vertices_one_rings(
        &mut self,
        vertex_id1: VertexId,
        vertex_id2: VertexId,
        flip_threshold_sqr: f32,
        marked_halfedges: &mut HashSet<HalfedgeId>,
        marked_vertices: &mut HashSet<VertexId>,
    ) -> MergeVerticesOneRing {
        #[cfg(feature = "instrumentation")]
        crate::set_current_op("merge_one_ring");
        #[cfg(feature = "instrumentation")]
        crate::probe_chain_begin(self);
        let vertex1 = *unwrap_or_return!(
            self.vertices.get(vertex_id1),
            "Vertex not found",
            MergeVerticesOneRing::default()
        );
        let vertex2 = *unwrap_or_return!(
            self.vertices.get(vertex_id2),
            "Vertex not found",
            MergeVerticesOneRing::default()
        );

        if vertex_id1 == vertex_id2 {
            error!("Cannot merge a vertex with itself");
            return MergeVerticesOneRing::default();
        }

        let one_ring_he_ids1 = vertex1.one_ring(self).collect_vec();
        let mut one_ring_he_ids2 = vertex2
            .one_ring(self)
            // halfedges are reversed, so twin halfedges are actually the next halfedges.
            // also existence is checked in `one_ring()`.
            .filter_map(|he_id| {
                // `one_ring` derives the ids from the outgoing star of the vertex via
                // twins; a dead id here (stale seed or dangling twin) would panic on
                // the index below, so skip it defensively.
                let he = self.halfedges.get(he_id)?;
                he.twin.or_else(error_none!("Twin not found"))
            })
            .collect_vec();
        one_ring_he_ids2.reverse();

        if one_ring_he_ids1.len() < 3 || one_ring_he_ids2.len() < 3 {
            error!(
                "One rings are too small. One ring of {vertex_id1:?} = {}; one ring of {vertex_id2:?} = {}",
                one_ring_he_ids1.len(),
                one_ring_he_ids2.len()
            );
            return MergeVerticesOneRing::default();
        }

        // `one_ring` derives the ids from the outgoing star of the vertex via twins; a dead id
        // (stale seed or dangling twin) is skipped.
        let ring_end_vertices = |he_ids: &[HalfedgeId]| {
            let mut v_ids = he_ids
                .iter()
                .filter_map(|he_id| self.halfedges.get(*he_id).map(|he| he.end_vertex))
                .collect_vec();
            v_ids.rotate_right(1);
            v_ids
        };
        let one_ring_v_ids1 = ring_end_vertices(&one_ring_he_ids1);
        let mut one_ring_v_ids2 = ring_end_vertices(&one_ring_he_ids2);

        if one_ring_v_ids1.len() < 3 || one_ring_v_ids2.len() < 3 {
            error!("One rings are too small");
            return MergeVerticesOneRing::default();
        }

        // Rim taxonomy: measure the boundary the op opens and closes. Test-only; records one
        // probe per band merge into a thread-local.
        #[cfg(test)]
        let mut test_rim_probe = rim_probe::RimProbe::default();
        #[cfg(test)]
        {
            test_rim_probe.before = rim::boundary_halves(self);
            test_rim_probe.parallel_before = rim_probe::count_parallel_edges(self);
        }

        // The boundary as the op found it, by vertex pair, for the hole closers. It has to be
        // taken here, before the coincident fusion below: the fusion removes faces, so a
        // snapshot taken after it already *contains* that hole and the closers see nothing to
        // close - `MANIFOLD BROKE at merge VertexId(36v1)+VertexId(415v1) (boundary 0 -> 4)`
        // is exactly that case. Nothing between the top of this function and this line
        // touches the mesh.
        //
        // Both snapshots, and the deltas taken against them, cover only the region within
        // three edges of the two vertices, which holds everything the op touches: the stars
        // and the strip are one ring out, the fusion's and the cleanup's face removals around
        // ring vertices reach two, and merging a ring vertex with its border neighbours
        // (`merge_remaining_unconnected`) reaches the faces one ring further. The merge runs
        // hundreds of times per stroke, so whole-mesh scans here made it grow with the mesh.
        // A region that turned out too small could only leave a hole open, never seal a
        // border (see `rim::RegionBoundary`), and the test build checks every merge against
        // the whole-mesh scans.
        let region = self.vertices_around(&[vertex_id1, vertex_id2], 3);
        let boundary_at_entry = rim::RegionBoundary::new(self, &region);

        // The same boundary by halfedge, for the band's rim `H = boundary(after) \
        // boundary(before)`: `remove_face` detaches most rim members in place instead of
        // freeing them, so the twins of the removed halfedges are *not* the rim the op has to
        // close (see `rim::opened_rim`). Like the closers' snapshot it is taken before the
        // fusion, so whatever boundary the fusion opens is part of the rim the band closes: a
        // snapshot after it can cut a closed rim into an open chain that no band can consume.
        let boundary_before = rim::boundary_halves_around(self, &region);
        #[cfg(test)]
        let whole_mesh_boundary_before = rim::boundary_halves(self);

        // The vertex merges and splits this op makes, so the snapshot can be carried to the
        // closers (see `VertexRenames`).
        let mut renames = VertexRenames::default();
        let mut record = MergeRecord::new(marked_halfedges, marked_vertices);

        // Weld position-coincident cross-ring vertices before planning the band.
        //
        // When the two sheets touch (morphological close / collision merge), a
        // ring vertex of `vertex_id1` and a ring vertex of `vertex_id2` can share
        // a position while remaining distinct ids. Both rings then pass through
        // the same point, and any band spanning that zero-width seam contains two
        // faces over the same geometric triangle — a "flap" by the position-based
        // `faces_share_all_vertices`. `cleanup_bookkeeping` deletes the flap,
        // leaving a hole that nothing re-fills (the intermittent weld-hole
        // defect, ~1 in 12 runs, boundary halfedges left open).
        //
        // Fusing each coincident pair turns the two rings into the well-tested
        // shared-vertex case: the band is split at the shared vertex and the seam
        // is never spanned by a degenerate face. The position comparison is exact,
        // matching `faces_share_all_vertices`, so this fires exactly where the
        // flap deletion would have.
        let mut coincident_pairs = Vec::new();
        for &v_id1 in &one_ring_v_ids1 {
            for &v_id2 in &one_ring_v_ids2 {
                if v_id1 != v_id2
                    && self
                        .positions
                        .get(v_id1)
                        .is_some_and(|p1| self.positions.get(v_id2) == Some(p1))
                {
                    coincident_pairs.push((v_id1, v_id2));
                }
            }
        }
        // Which ring-2 vertex was fused into which ring-1 vertex. A ring-2 vertex coincident
        // with several ring-1 vertices is fused once, into the first; the later pairs find it
        // gone and must not overwrite that.
        let mut fused_into: HashMap<VertexId, VertexId> = HashMap::default();
        for (v1_id, v2_id) in &coincident_pairs {
            if !self.vertices.contains_key(*v1_id) || !self.vertices.contains_key(*v2_id) {
                continue;
            }

            tracing::debug!("fusing coincident ring vertices {v1_id:?} and {v2_id:?}");
            #[cfg(any(test, feature = "instrumentation"))]
            COINCIDENT_FUSIONS.with(|count| count.set(count.get() + 1));
            // Survivor is the ring-1 vertex, so the fused id stays on both rings
            // and shows up in `shared_v_ids` below.
            let fused = self.merge_vertices([*v1_id, *v2_id]);
            let result = &mut record.result;
            result.removed_faces.extend(fused.removed_faces);
            result.removed_halfedges.extend(fused.removed_halfedges);
            // `fused.removed_vertices` is `[*v2_id]` plus any vertex the face removal isolated.
            for v_id in fused.removed_vertices {
                if !result.removed_vertices.contains(&v_id) {
                    result.removed_vertices.push(v_id);
                }
            }

            if !self.vertices.contains_key(*v2_id) {
                fused_into.insert(*v2_id, *v1_id);
                renames.absorb(*v2_id, *v1_id);
            }
        }
        for v_id in &mut one_ring_v_ids2 {
            if let Some(survivor) = fused_into.get(v_id) {
                *v_id = *survivor;
            }
        }

        let one_ring_he_set1: HashSet<HalfedgeId> = one_ring_he_ids1.iter().copied().collect();
        let one_ring_he_set2: HashSet<HalfedgeId> = one_ring_he_ids2.iter().copied().collect();
        let shared_he_ids: HashSet<HalfedgeId> = one_ring_he_set1
            .intersection(&one_ring_he_set2)
            .copied()
            .collect();

        let one_ring_v_set1: HashSet<VertexId> = one_ring_v_ids1.iter().copied().collect();
        let one_ring_v_set2: HashSet<VertexId> = one_ring_v_ids2.iter().copied().collect();
        let shared_v_ids: HashSet<VertexId> = one_ring_v_set1
            .intersection(&one_ring_v_set2)
            .copied()
            .collect();

        tracing::debug!("ring1 (v_ids): {one_ring_v_ids1:?}");
        tracing::debug!("ring2 (v_ids): {one_ring_v_ids2:?}");
        tracing::debug!("shared_v_ids: {shared_v_ids:?}");

        if self.check_and_flip_single_shared_he(
            &shared_he_ids,
            flip_threshold_sqr,
            &mut renames,
            &mut record.result,
        ) {
            // That flip-plus-collapse removes faces, so it leaves the same kind of hole the band
            // does.
            if merge_nets_enabled() {
                self.close_merge_opened_holes(
                    &one_ring_v_ids1,
                    &one_ring_v_ids2,
                    &boundary_at_entry,
                    &renames,
                    &mut record,
                );
            }

            return self.finish_merge(record);
        }

        self.remove_vertex_faces(vertex_id1, &mut record.result);

        #[cfg(test)]
        {
            test_rim_probe.h1 = rim::opened_rim(self, &test_rim_probe.before);
        }
        let star_h1_raw = rim::opened_rim_around(self, &region, &boundary_before);
        #[cfg(test)]
        assert_eq!(
            star_h1_raw,
            rim::opened_rim(self, &whole_mesh_boundary_before),
            "the first star's rim reaches past the merge region"
        );

        self.remove_vertex_faces(vertex_id2, &mut record.result);

        #[cfg(test)]
        {
            test_rim_probe.rim = rim::opened_rim(self, &test_rim_probe.before);
            let h1 = test_rim_probe.h1.clone();
            test_rim_probe.h2 = test_rim_probe.rim.difference(&h1).copied().collect();
        }

        let star_h1: HashSet<HalfedgeId> = star_h1_raw
            .iter()
            .copied()
            .filter(|he_id| self.halfedges.contains_key(*he_id))
            .collect();

        #[cfg(feature = "rerun")]
        self.log_rerun();

        let strip_faces = match self.find_already_connected_strip(
            &one_ring_v_ids1,
            &one_ring_v_ids2,
            &shared_v_ids,
        ) {
            Some(strip_faces) => strip_faces,
            None => {
                tracing::error!("Error in find_already_connected_strip");

                // The stars are already out by this point, so returning here leaves their rim
                // open: the op tears the mesh and does not put it back. No band can be
                // planned, but the surface can stay closed.
                if merge_nets_enabled() {
                    self.close_merge_opened_holes(
                        &one_ring_v_ids1,
                        &one_ring_v_ids2,
                        &boundary_at_entry,
                        &renames,
                        &mut record,
                    );
                }

                return self.finish_merge(record);
            }
        };

        #[cfg(feature = "rerun")]
        self.log_faces_rerun("already_connected", &strip_faces);

        // Plan the band from the rim the removals opened. The strip faces lie inside that
        // rim, so they go first.
        self.remove_faces(&strip_faces, &mut record.result);
        let (rim, h1, h2) = self.strip_first_rim(&region, &boundary_before, &star_h1);
        #[cfg(test)]
        assert_eq!(
            rim,
            rim::opened_rim(self, &whole_mesh_boundary_before),
            "the stars' and the strip's rim reaches past the merge region"
        );
        let plan = rim_planner::plan(self, &rim, &h1, &h2);
        for note in &plan.unhandled {
            tracing::debug!("rim-first planner unhandled: {note}");
        }
        // Only a plan that closes exactly the rim is built. Without one - in practice an
        // open chain, a rim that runs into a pre-existing border - nothing is planned, and
        // the cleanup and the remaining-unconnected merge below repair the region.
        let planned_faces = if plan.is_complete(self, &rim) {
            plan.faces
        } else {
            Vec::new()
        };

        for pf in &planned_faces {
            tracing::debug!(
                "planned {:?} v1={:?} a={:?} b={:?}",
                pf.order,
                pf.v1,
                pf.new_he_v1,
                pf.new_he_v2
            );
        }

        #[cfg(test)]
        {
            test_rim_probe
                .rim
                .extend(rim::opened_rim(self, &test_rim_probe.before));
            test_rim_probe.classify(self);
            test_rim_probe.measure_rim_plan(self);
        }

        self.add_planned_faces(
            planned_faces,
            &one_ring_v_set1,
            &one_ring_v_set2,
            &mut record,
        );

        #[cfg(test)]
        {
            test_rim_probe.finish(self);
        }

        if merge_nets_enabled() {
            self.weld_parallel_edges(
                ring_region(
                    &one_ring_v_ids1,
                    &one_ring_v_ids2,
                    &record.result.added_vertices,
                ),
                &mut record,
            );
        }

        #[cfg(feature = "rerun")]
        self.log_rerun();

        self.cleanup_bookkeeping(
            &one_ring_v_ids1,
            &one_ring_v_ids2,
            &mut renames,
            &mut record,
        );

        self.merge_remaining_unconnected(
            one_ring_v_ids1.clone(),
            one_ring_v_ids2.clone(),
            &mut renames,
            &mut record,
        );

        // Again after the cleanup: a pair can still be parallel here that was not
        // weldable above, because the two faces hung off the *same* side of the
        // edge until the cleanup rebuilt one of them.
        if merge_nets_enabled() {
            self.weld_parallel_edges(
                ring_region(
                    &one_ring_v_ids1,
                    &one_ring_v_ids2,
                    &record.result.added_vertices,
                ),
                &mut record,
            );

            self.close_merge_opened_holes(
                &one_ring_v_ids1,
                &one_ring_v_ids2,
                &boundary_at_entry,
                &renames,
                &mut record,
            );
        }

        self.smooth_vertices(ring_region(
            &one_ring_v_ids1,
            &one_ring_v_ids2,
            &record.result.added_vertices,
        ));

        #[cfg(test)]
        {
            test_rim_probe.finish_end(self);
            rim_probe::push(test_rim_probe);
        }

        self.finish_merge(record)
    }

    /// Common op-end path: drop the removed ids from the caller's marked sets
    /// ([`MergeRecord::finish`]), run the integrity/hole inspectors and push the mesh onto
    /// the state-history ring (instrumentation), then return the result. Every exit of the
    /// op that has touched the mesh goes through here, so the ring stays continuous with the
    /// journal (resume continuity) and no corruption can skip the checks.
    fn finish_merge(&self, record: MergeRecord) -> MergeVerticesOneRing {
        let result = record.finish();

        #[cfg(feature = "instrumentation")]
        if self.probe_chain_integrity("merge_vertices_one_rings") {
            crate::state_history_push(self, "merge_vertices_one_rings");
        }

        result
    }

    fn merge_remaining_unconnected(
        &mut self,
        mut one_ring_v_ids1: Vec<VertexId>,
        mut one_ring_v_ids2: Vec<VertexId>,
        renames: &mut VertexRenames,
        record: &mut MergeRecord,
    ) {
        while let Some(v_id) = one_ring_v_ids1.pop() {
            let Some(vert) = self.vertices.get(v_id) else {
                continue;
            };

            let mut vertices = vec![];

            for he_id in vert.boundary_halfedgdes(self) {
                let Some(he) = self.halfedges.get(he_id) else {
                    error!("Halfedge not found: {he_id:?}");
                    continue;
                };

                vertices.push(he.end_vertex);
            }

            if !vertices.is_empty() {
                one_ring_v_ids1.retain(|v_id| !vertices.contains(v_id));
                one_ring_v_ids2.retain(|v_id| !vertices.contains(v_id));

                // `merge_vertices` keeps the first live vertex and absorbs the other live ones.
                // Only those are renames: `removed_vertices` also lists vertices the face
                // removal left isolated, whose edges are gone rather than moved.
                let members = vertices
                    .iter()
                    .copied()
                    .unique()
                    .filter(|v_id| self.vertices.contains_key(*v_id))
                    .collect_vec();

                let MergeVertices {
                    removed_vertices,
                    removed_halfedges,
                    removed_faces,
                } = self.merge_vertices(vertices);

                if let Some((&survivor, absorbed)) = members.split_first() {
                    for &absorbed in absorbed {
                        if !self.vertices.contains_key(absorbed) {
                            renames.absorb(absorbed, survivor);
                        }
                    }
                }

                // The cleanup ran before this, so a merged vertex or a removed edge can be
                // one it split off or the band added.
                for v_id in removed_vertices {
                    record.vertex_removed(v_id);
                }
                for he_id in removed_halfedges {
                    record.halfedge_removed(he_id);
                }
                for face_id in removed_faces {
                    record.face_removed(face_id);
                }
            }
        }
    }

    /// The faces that already join the two rings: the first contiguous strip of them, which
    /// the band replaces. `None` when the connectivity around the rings is broken.
    fn find_already_connected_strip(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        shared_v_ids: &HashSet<VertexId>,
    ) -> Option<Vec<FaceId>> {
        let mut connected_v_ids = HashSet::default();
        let mut already_connected_face_ids = vec![];

        // Only the first strip of already existing faces is taken; a second one stays, and
        // the band spans it (`is_complete` refuses a plan that would duplicate its edges).
        let mut strip_found = false;

        for ((idx1, &v_id1), (idx2, &v_id2)) in one_ring_v_ids1
            .iter()
            .enumerate()
            .cartesian_product(one_ring_v_ids2.iter().enumerate())
        {
            if shared_v_ids.contains(&v_id1) || shared_v_ids.contains(&v_id2) {
                continue;
            }

            if v_id1 == v_id2 {
                connected_v_ids.insert(v_id1);
                continue;
            }

            if connected_v_ids.contains(&v_id1) && connected_v_ids.contains(&v_id2) {
                continue;
            }

            // The fan of the first strip is already extended over the whole strip
            // by `find_connected_triangle_fans` below, so any fan found from here
            // on belongs to a separate strip. Leave its faces in place and let the
            // new faces span it instead.
            if strip_found {
                continue;
            }

            if let Some(he_id) = self.halfedge_from_to(v_id1, v_id2) {
                let pairing = match self.find_triangle_fan(
                    he_id,
                    idx1,
                    idx2,
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &mut already_connected_face_ids,
                ) {
                    TriangleFan::Spanning(pairing) => pairing,
                    // The halfedge only touches the rings from the outside, so it
                    // doesn't connect them.
                    TriangleFan::NotSpanning => continue,
                    TriangleFan::Failed => return None,
                };

                #[cfg(feature = "rerun")]
                pairing.log_rerun(
                    "already_connected_init",
                    [one_ring_v_ids1, one_ring_v_ids2],
                    self,
                );

                let mut connected_pairings = self.find_connected_triangle_fans(
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &pairing,
                    -1,
                    &mut already_connected_face_ids,
                );

                connected_pairings.push(pairing.clone());

                connected_pairings.extend(self.find_connected_triangle_fans(
                    one_ring_v_ids1,
                    one_ring_v_ids2,
                    &pairing,
                    1,
                    &mut already_connected_face_ids,
                ));

                for pairing in &connected_pairings {
                    for v_id in pairing.all_vertex_ids([one_ring_v_ids1, one_ring_v_ids2]) {
                        connected_v_ids.insert(v_id);
                    }
                }

                strip_found = true;
            }
        }

        Some(already_connected_face_ids)
    }

    fn find_connected_triangle_fans(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        pairing: &Pairing,
        idx_step: isize,
        connected_face_ids: &mut Vec<FaceId>,
    ) -> Vec<Pairing> {
        let (single_v_ids, other_v_ids) = pairing.rings([one_ring_v_ids1, one_ring_v_ids2]);

        let next_single_idx = (pairing.single_idx_in_range + single_v_ids.len())
            .wrapping_add_signed(idx_step)
            % single_v_ids.len();
        let next_v_id = single_v_ids[next_single_idx];

        let mut other_idx = if idx_step < 0 {
            *pairing.other_range.start()
        } else {
            *pairing.other_range.end()
        };
        other_idx %= other_v_ids.len();

        let current_single_v_id = single_v_ids[pairing.single_idx_in_range];

        let mut pairings = vec![];

        if self
            .face_with_vertices(next_v_id, current_single_v_id, other_v_ids[other_idx])
            .is_some()
        {
            // swap single range and other range
            let mut current_pairing = Pairing::new_triangle(
                1 - pairing.single_range_idx,
                other_idx,
                next_single_idx,
                pairing.single_idx_in_range,
                single_v_ids.len(),
            );

            self.extend_triangle_pairing_fan(
                one_ring_v_ids1,
                one_ring_v_ids2,
                &mut current_pairing,
                connected_face_ids,
            );

            #[cfg(feature = "rerun")]
            current_pairing.log_rerun(
                format!("extended/step_{}", idx_step),
                [one_ring_v_ids1, one_ring_v_ids2],
                self,
            );

            let next_pairings = self.find_connected_triangle_fans(
                one_ring_v_ids1,
                one_ring_v_ids2,
                &current_pairing,
                idx_step,
                connected_face_ids,
            );

            pairings.push(current_pairing);
            pairings.extend(next_pairings);
        }

        pairings
    }

    /// Finds the fan of already existing faces that connects the two one rings
    /// across the halfedge `he_id` (which runs from a ring 1 vertex to a ring 2
    /// vertex). See [`TriangleFan`] for the possible outcomes.
    #[allow(clippy::too_many_arguments)]
    fn find_triangle_fan(
        &self,
        he_id: HalfedgeId,
        idx1: usize,
        idx2: usize,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        connected_face_ids: &mut Vec<FaceId>,
    ) -> TriangleFan {
        let he = unwrap_or_return!(
            self.halfedges.get(he_id),
            "Halfedge not found",
            TriangleFan::Failed
        );

        let mut current_pairing = Pairing {
            single_range_idx: 0,
            single_idx_in_range: idx1,
            other_range: idx2..=idx2,
        };

        // might not exist. we removed some faces (neighbors).
        let Some(opposite_v_id) = he.opposite_vertex(self) else {
            // The face on this side was one of the removed neighbor faces, so the
            // halfedge does span the region between the rings.
            return TriangleFan::Spanning(current_pairing);
        };

        if !self.create_triangle_pairing(
            one_ring_v_ids1,
            one_ring_v_ids2,
            idx1,
            idx2,
            opposite_v_id,
            &mut current_pairing,
        ) {
            let twin_id = unwrap_or_return!(he.twin, "Twin not found", TriangleFan::Failed);
            let twin = unwrap_or_return!(
                self.halfedges.get(twin_id),
                "Twin not found",
                TriangleFan::Failed
            );

            // might not exist. we removed some faces (neighbors).
            let Some(opposite_v_id) = twin.opposite_vertex(self) else {
                return TriangleFan::Spanning(current_pairing);
            };

            if !self.create_triangle_pairing(
                one_ring_v_ids1,
                one_ring_v_ids2,
                idx1,
                idx2,
                opposite_v_id,
                &mut current_pairing,
            ) {
                return TriangleFan::NotSpanning;
            }
        }

        self.extend_triangle_pairing_fan(
            one_ring_v_ids1,
            one_ring_v_ids2,
            &mut current_pairing,
            connected_face_ids,
        );

        TriangleFan::Spanning(current_pairing)
    }

    fn extend_triangle_pairing_fan(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        current_pairing: &mut Pairing,
        connected_face_ids: &mut Vec<FaceId>,
    ) {
        let (single_ids, other_ids) = current_pairing.rings([one_ring_v_ids1, one_ring_v_ids2]);

        let other_len = other_ids.len();
        let single_v_id = single_ids[current_pairing.single_idx_in_range % single_ids.len()];

        // Walk backwards from the start of other_range
        let mut range_start = *current_pairing.other_range.start();

        let start_face_id = unwrap_or_return!(
            self.face_with_vertices(
                single_v_id,
                other_ids[range_start % other_len],
                other_ids[*current_pairing.other_range.end() % other_len],
            ),
            "failed to find start face for pairing fan"
        );
        connected_face_ids.push(start_face_id);

        // Cap the number of steps so a complete face fan around the single vertex
        // can't make the walk wrap around the ring indefinitely.
        let mut steps = 0;
        loop {
            // Wrap backwards (using saturating to avoid underflow; handle wrap via modular arithmetic)
            let prev_idx = (range_start + other_len - 1) % other_len;
            let prev_v_id = other_ids[prev_idx];

            let boundary_v_id = other_ids[range_start % other_len];
            if let Some(face_id) = self.face_with_vertices(single_v_id, prev_v_id, boundary_v_id) {
                // Adjust range_start to go one step back; keep values un-modded so the
                // range arithmetic stays consistent with how ConnectPair::new handles wrapping.
                if range_start == 0 {
                    range_start = other_len - 1;
                } else {
                    range_start -= 1;
                }
                current_pairing.other_range = range_start..=*current_pairing.other_range.end();
                connected_face_ids.push(face_id);

                steps += 1;
                if steps >= other_len {
                    error!("Pairing fan wrapped around the entire one ring");
                    break;
                }
            } else {
                break;
            }
        }

        // Walk forwards from the end of other_range
        let mut range_end = *current_pairing.other_range.end();
        // Cap the number of steps so a complete face fan around the single vertex
        // can't make the walk wrap around the ring indefinitely.
        let mut steps = 0;
        loop {
            let next_idx = (range_end + 1) % other_len;
            let next_v_id = other_ids[next_idx];

            let boundary_v_id = other_ids[range_end % other_len];
            if let Some(face_id) = self.face_with_vertices(single_v_id, boundary_v_id, next_v_id) {
                range_end += 1;
                current_pairing.other_range = *current_pairing.other_range.start()..=range_end;
                connected_face_ids.push(face_id);

                steps += 1;
                if steps >= other_len {
                    error!("Pairing fan wrapped around the entire one ring");
                    break;
                }
            } else {
                break;
            }
        }
    }

    fn create_triangle_pairing(
        &self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        idx1: usize,
        idx2: usize,
        third_v_id: VertexId,
        current_pairing: &mut Pairing,
    ) -> bool {
        let (idx1, idx2) =
            if let Some(idx) = one_ring_v_ids1.iter().position(|v_id| *v_id == third_v_id) {
                current_pairing.single_range_idx = 1;
                current_pairing.single_idx_in_range = idx2;

                (idx, idx1)
            } else if let Some(idx) = one_ring_v_ids2.iter().position(|v_id| *v_id == third_v_id) {
                (idx, idx2)
            } else {
                return false;
            };

        let other_len = current_pairing
            .rings([one_ring_v_ids1, one_ring_v_ids2])
            .1
            .len();

        *current_pairing = Pairing::new_triangle(
            current_pairing.single_range_idx,
            current_pairing.single_idx_in_range,
            idx1,
            idx2,
            other_len,
        );

        true
    }

    fn cleanup_bookkeeping(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        renames: &mut VertexRenames,
        record: &mut MergeRecord,
    ) {
        for vertex_id in one_ring_v_ids1.iter().chain(one_ring_v_ids2).copied() {
            if !self.vertices.contains_key(vertex_id) {
                continue;
            }

            let cleanup = self.make_vertex_neighborhood_manifold(vertex_id);

            // The cleanup only ever splits `vertex_id` and the vertices it split off, so every
            // added vertex descends from it.
            for &added_v_id in &cleanup.added_vertices {
                renames.split(vertex_id, added_v_id);
            }

            if !cleanup.added_vertices.is_empty() {
                record.marked_vertices.insert(vertex_id);
            }

            record.marked_vertices.extend(&cleanup.added_vertices);
            record.result.added_vertices.extend(cleanup.added_vertices);

            for v_id in cleanup.removed_vertices {
                record.vertex_removed(v_id);
            }
            for he_id in cleanup.removed_halfedges {
                record.halfedge_removed(he_id);
            }
            for face_id in cleanup.removed_faces {
                record.face_removed(face_id);
            }
        }
    }

    /// Remove the given faces, recording the removal on `result`.
    fn remove_faces(&mut self, face_ids: &[FaceId], result: &mut MergeVerticesOneRing) {
        for &face_id in face_ids {
            let (v_ids, he_ids) = self.remove_face(face_id);
            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(he_ids);
            result.removed_vertices.extend(v_ids);
        }
    }

    /// The live rim after the already-connected strip is torn out, split into
    /// the two merged stars' halves (the strip's own halves become the third
    /// category the planner classifies).
    fn strip_first_rim(
        &self,
        region: &[VertexId],
        boundary_before: &HashSet<HalfedgeId>,
        star_h1: &HashSet<HalfedgeId>,
    ) -> (
        HashSet<HalfedgeId>,
        HashSet<HalfedgeId>,
        HashSet<HalfedgeId>,
    ) {
        let rim = rim::opened_rim_around(self, region, boundary_before);
        let h1: HashSet<HalfedgeId> = star_h1
            .iter()
            .copied()
            .filter(|he_id| self.halfedges.contains_key(*he_id))
            .collect();
        let h2: HashSet<HalfedgeId> = rim
            .iter()
            .copied()
            .filter(|he_id| !h1.contains(he_id))
            .collect();
        (rim, h1, h2)
    }

    fn add_planned_faces(
        &mut self,
        planned_faces: Vec<PlannedFace>,
        one_ring_set1: &HashSet<VertexId>,
        one_ring_set2: &HashSet<VertexId>,
        record: &mut MergeRecord,
    ) {
        let mut prev_he = None;

        for planned_face in planned_faces {
            let inserted = if let Some(prev_he) = prev_he {
                planned_face.add_to_mesh_graph_and_he(self, prev_he, record)
            } else {
                planned_face.add_to_mesh_graph(self, record)
            };

            let Some((next_prev_he, inserted)) = inserted else {
                // Depending on the topology inserting a face can fail sometimes.
                // This is fine: the insertion transaction rolled back whatever it
                // had created, so the mesh is exactly as it was before this face.
                // Record the skip - a systematically skipped face must not hide
                // as a successful merge.
                tracing::debug!("skipped planned face {planned_face:?}");
                #[cfg(test)]
                rim_probe::note_skipped_face();
                prev_he = None;
                continue;
            };

            prev_he = next_prev_he;

            for he_id in inserted.halfedge_ids.iter().copied() {
                // just inserted => must exist
                let he = self.halfedges[he_id];
                let start_v_id =
                    unwrap_or_return!(he.start_vertex(self), "Couldn't find start vertex");
                let end_v_id = he.end_vertex;

                if one_ring_set1.contains(&start_v_id) && one_ring_set2.contains(&end_v_id)
                    || one_ring_set2.contains(&start_v_id) && one_ring_set1.contains(&end_v_id)
                {
                    record.marked_halfedges.insert(he_id);
                }
            }

            record.result.added_halfedges.extend(inserted.halfedge_ids);
            record.result.added_faces.push(inserted.face_id);
        }
    }

    /// Fuse *parallel* edges left behind by the new faces.
    ///
    /// `add_or_get_boundary_edge` only ever reuses a free half in the direction
    /// it was asked for. When a planned face needs `a -> b` and that direction
    /// already carries a face, it adds a second, parallel edge between the same
    /// two vertices instead. That is fine while the region is still being
    /// rebuilt - a later face or the neighborhood cleanup usually claims the
    /// free halves - but when nothing does, the two halves stay face-less
    /// forever. The mesh then reports them as boundary (a "hole" with no face
    /// missing from it) and the two triangles that should share the edge each
    /// sit on their own copy of it.
    ///
    /// This is the sweep for that leftover: where two vertices are joined by
    /// more than one edge but exactly one face hangs off each direction, the two
    /// faced halves become each other's twin and the orphans are dropped. Any
    /// other shape is left alone - more than one face per direction is a
    /// genuinely non-manifold edge, which is the neighborhood cleanup's job.
    fn weld_parallel_edges(
        &mut self,
        region_v_ids: impl IntoIterator<Item = VertexId>,
        record: &mut MergeRecord,
    ) {
        let mut seen = HashSet::<VertexId>::default();

        for v_id in region_v_ids {
            if !seen.insert(v_id) || !self.vertices.contains_key(v_id) {
                continue;
            }

            let Some(out_he_ids) = self.outgoing_halfedges.get(v_id).cloned() else {
                continue;
            };

            // End vertices this one reaches more than once, in a stable order.
            let mut ends = Vec::new();
            for he_id in &out_he_ids {
                if let Some(he) = self.halfedges.get(*he_id)
                    && !ends.contains(&he.end_vertex)
                {
                    ends.push(he.end_vertex);
                }
            }

            for other_v_id in ends {
                let forward = self.halfedges_from_to(v_id, other_v_id);
                let backward = self.halfedges_from_to(other_v_id, v_id);

                if forward.len() + backward.len() <= 2 {
                    continue;
                }

                let faced = |mesh: &Self, ids: &[HalfedgeId]| -> Vec<HalfedgeId> {
                    ids.iter()
                        .copied()
                        .filter(|he_id| {
                            mesh.halfedges
                                .get(*he_id)
                                .is_some_and(|he| he.face.is_some())
                        })
                        .collect()
                };

                let faced_forward = faced(self, &forward);
                let faced_backward = faced(self, &backward);

                if faced_forward.len() != 1 || faced_backward.len() != 1 {
                    continue;
                }

                let keep_forward = faced_forward[0];
                let keep_backward = faced_backward[0];

                self.halfedges[keep_forward].twin = Some(keep_backward);
                self.halfedges[keep_backward].twin = Some(keep_forward);

                for he_id in forward.into_iter().chain(backward) {
                    if he_id == keep_forward || he_id == keep_backward {
                        continue;
                    }
                    // Only the orphans go; anything with a face was counted above.
                    if self
                        .halfedges
                        .get(he_id)
                        .is_some_and(|he| he.face.is_none())
                    {
                        // An orphan is often the free half a planned face just added.
                        self.remove_only_halfedge(he_id);
                        record.halfedge_removed(he_id);
                    }
                }

                self.reseed_outgoing_if_dead(v_id);
                self.reseed_outgoing_if_dead(other_v_id);
            }
        }
    }

    /// Closes the holes this merge's removals opened, wherever they are.
    ///
    /// The search region is the merge's own rings plus the endpoints of every boundary edge
    /// the op opened since entry: a hole can sit outside the rings (the coincident fusion,
    /// the star removal and the strip walk all reach further).
    ///
    /// Both the search and the snapshot cover only the region around the merge
    /// ([`rim::RegionBoundary`]); the test build checks that a whole-mesh scan finds the
    /// same opened boundary.
    ///
    /// "Opened" is keyed by vertex pair, not halfedge id: the merge deletes border edges and
    /// re-creates them under new ids, and an id-keyed set would take such an edge for one the
    /// merge opened and seal a border that is meant to stay open. The walker is the collapse
    /// closer's ([`MeshGraph::boundary_loop_from`]), so both ops share one definition of
    /// "a loop this op opened"; it follows opened edges only, of any length, and
    /// [`MeshGraph::close_loop_with_fan`] refuses cuts that would fold the loop. The snapshot
    /// is carried through the vertex merges and splits the op made so far (`renames`), or the
    /// old border edges at those vertices would look opened.
    ///
    /// A three-halfedge loop around a lone triangle is emptied rather than closed: the
    /// triangle is removed (see [`MeshGraph::lone_face_behind`]).
    fn close_merge_opened_holes(
        &mut self,
        one_ring_v_ids1: &[VertexId],
        one_ring_v_ids2: &[VertexId],
        boundary_at_entry: &rim::RegionBoundary,
        renames: &VertexRenames,
        record: &mut MergeRecord,
    ) {
        let boundary_at_entry = &boundary_at_entry.resolve(renames);

        let opened =
            self.opened_boundary_pairs_around(boundary_at_entry.region(), boundary_at_entry);
        #[cfg(test)]
        assert_eq!(
            opened,
            self.opened_boundary_pairs(&boundary_at_entry.whole_mesh),
            "the merge opened boundary past its region"
        );

        let mut closer_region = ring_region(
            one_ring_v_ids1,
            one_ring_v_ids2,
            &record.result.added_vertices,
        );
        let mut in_region: HashSet<VertexId> = closer_region.iter().copied().collect();

        for v_id in vertices_of_pairs(opened) {
            if in_region.insert(v_id) {
                closer_region.push(v_id);
            }
        }

        // The faces added below take two halves out of the boundary and add the fan's
        // diagonal, so each sweep re-walks until nothing closes.
        loop {
            let mut progressed = false;

            for &start_v in &closer_region {
                if !self.vertices.contains_key(start_v) {
                    continue;
                }

                let Some(loop_hes) = self.boundary_loop_from(start_v, boundary_at_entry) else {
                    continue;
                };

                if let Some(face_id) = self.lone_face_behind(&loop_hes) {
                    let removed = self.remove_face(face_id);
                    record.face_removed_with(face_id, removed);
                    progressed = true;
                    continue;
                }

                if let Some(added) = self.close_loop_with_fan(&loop_hes) {
                    record.result.added_faces.push(added.face_id);
                    record.result.added_halfedges.extend(added.halfedge_ids);
                    progressed = true;
                }
            }

            if !progressed {
                break;
            }
        }
    }

    /// The vertices within `hops` edges of `seeds`, seeds first, in breadth-first order over
    /// the raw outgoing lists - so every fan of a pinched vertex is followed, not just the
    /// one `one_ring` sees.
    fn vertices_around(&self, seeds: &[VertexId], hops: usize) -> Vec<VertexId> {
        let mut region: Vec<VertexId> = Vec::new();
        let mut seen: HashSet<VertexId> = HashSet::default();
        for &seed in seeds {
            if seen.insert(seed) {
                region.push(seed);
            }
        }

        let mut frontier_start = 0;
        for _ in 0..hops {
            let frontier_end = region.len();
            for idx in frontier_start..frontier_end {
                let Some(out_he_ids) = self.outgoing_halfedges.get(region[idx]) else {
                    continue;
                };
                for he_id in out_he_ids {
                    if let Some(he) = self.halfedges.get(*he_id)
                        && seen.insert(he.end_vertex)
                    {
                        region.push(he.end_vertex);
                    }
                }
            }
            frontier_start = frontier_end;
        }

        region
    }

    /// Remove every face incident to `vertex_id`, walking the vertex's raw
    /// outgoing-halfedge list rather than [`Vertex::faces`].
    ///
    /// `faces()` follows one fan from the seed, so a pinched vertex (its star
    /// split into more than one fan) leaves the other fans' faces behind - and
    /// `H`, the boundary delta these removals produce, would then describe only
    /// part of the hole while the vertex stays attached. Used once per merge
    /// vertex, in order, so the rim each star opens can be attributed to its
    /// side.
    fn remove_vertex_faces(&mut self, vertex_id: VertexId, result: &mut MergeVerticesOneRing) {
        let mut face_ids: Vec<FaceId> = Vec::new();
        for he_id in self
            .outgoing_halfedges
            .get(vertex_id)
            .cloned()
            .unwrap_or_default()
        {
            if let Some(face_id) = self.halfedges.get(he_id).and_then(|he| he.face)
                && !face_ids.contains(&face_id)
            {
                face_ids.push(face_id);
            }
        }

        self.remove_faces(&face_ids, result);

        // The star must be gone in full: no live outgoing halfedge of the
        // vertex may still claim a face. A survivor means the enumeration
        // missed a fan and `H` is a partial hole.
        debug_assert!(
            self.outgoing_halfedges
                .get(vertex_id)
                .is_none_or(|out_hes| out_hes.iter().all(|he_id| self
                    .halfedges
                    .get(*he_id)
                    .is_none_or(|he| he.face.is_none()))),
            "star removal left faces on {vertex_id:?}"
        );
    }

    /// Dismantles the face (if any) that still claims `he_id` as a chain member.
    /// Planned bridge faces re-file halfedges that an overlapping earlier merge's
    /// bridge/fan may still own; re-filing a live chain member corrupts the old
    /// face's chain (face-stealing), so such faces are dismantled before the
    /// re-file. Only faces inside the merged region are affected: a halfedge that
    /// is re-filed by a planned face always has its other side owned by a face of
    /// the same region, never by an outside surface.
    ///
    /// The face can be one this op's band inserted a moment earlier, so the removal goes
    /// through [`MergeRecord::face_removed_with`].
    fn dismantle_faced_member(&mut self, he_id: HalfedgeId, record: &mut MergeRecord) {
        if let Some(he) = self.halfedges.get(he_id)
            && let Some(face_id) = he.face
            && self.faces.contains_key(face_id)
        {
            let removed = self.remove_face(face_id);
            record.face_removed_with(face_id, removed);
        }
    }

    /// The `(start, end)` vertex pair of a live halfedge.
    fn halfedge_endpoints(&self, he_id: HalfedgeId) -> Option<(VertexId, VertexId)> {
        let he = self.halfedges.get(he_id)?;
        Some((he.start_vertex(self)?, he.end_vertex))
    }

    /// The live boundary half from `start` to `end`, if any.
    fn boundary_half_from_to(&self, start: VertexId, end: VertexId) -> Option<HalfedgeId> {
        self.halfedges_from_to(start, end)
            .into_iter()
            .find(|he_id| self.halfedges[*he_id].is_boundary())
    }

    /// Remove the edge pairs a failed planned-face insertion created. They are
    /// self-twinned and face-less by construction (nothing claims them before
    /// the face insertion runs), so removing both halves is safe.
    fn rollback_created_edges(&mut self, created: &mut Vec<HalfedgeId>) {
        for he_id in created.drain(..) {
            if self.halfedges.contains_key(he_id) {
                self.remove_only_halfedge_and_twin(he_id);
            }
        }
    }

    /// A pre-commit assertion: after a planned face was inserted, none of the
    /// edges created for it may be left face-less on both sides (wire). That
    /// can only happen if the insertion claimed a different half - a
    /// transaction bug.
    #[cfg(test)]
    fn debug_assert_created_edges_claimed(&self, created: &[HalfedgeId]) {
        for &he_id in created {
            let Some(he) = self.halfedges.get(he_id) else {
                continue;
            };
            let twin_faced = he
                .twin
                .and_then(|twin_id| self.halfedges.get(twin_id))
                .is_some_and(|twin| twin.face.is_some());
            debug_assert!(
                he.face.is_some() || twin_faced,
                "planned-face insertion left created edge {he_id:?} face-less on both sides"
            );
        }
    }

    /// A pre-commit assertion for the two inputs of `add_face_from_halfedges`:
    /// each must resolve to a boundary half with a twin and a resolvable start
    /// vertex. Otherwise the function's internal third-edge acquisition can
    /// create an edge and then fail on a missing twin, leaking an edge the
    /// caller's `created` bookkeeping never saw.
    #[cfg(test)]
    fn debug_assert_face_inputs_ready(&self, he_id1: HalfedgeId, he_id2: HalfedgeId) {
        for he_id in [he_id1, he_id2] {
            let resolved = self.boundary_he(he_id);
            debug_assert!(
                resolved.is_some(),
                "planned-face input {he_id:?} resolves to no boundary half"
            );
            if let Some(resolved) = resolved
                && let Some(he) = self.halfedges.get(resolved)
            {
                debug_assert!(
                    he.twin.is_some() && he.start_vertex(self).is_some(),
                    "planned-face input {resolved:?} is not insertion-ready (missing twin or start vertex)"
                );
            }
        }
    }

    #[instrument(skip_all)]
    fn flip_and_collapse_single_shared_edge_if_below_threshold(
        &mut self,
        single_shared_he_id: HalfedgeId,
        flip_threshold_sqr: f32,
        renames: &mut VertexRenames,
        result: &mut MergeVerticesOneRing,
    ) -> bool {
        // A lookup, not an index: the caller picks this edge out of a set built from the
        // ring lists collected *before* the coincident fusion above, which can have
        // consumed it (a fused pair removes the edge between it and its twin). Indexing a
        // dead id here aborted the whole merge; a dead id means the shared edge the set
        // counted is gone, so there is nothing to flip and the caller plans the band the
        // normal way.
        let he = *unwrap_or_return!(
            self.halfedges.get(single_shared_he_id),
            "Shared halfedge not found",
            false
        );

        let twin_id = unwrap_or_return!(he.twin, "Twin not found", false);
        let twin = unwrap_or_return!(self.halfedges.get(twin_id), "Twin not found", false);

        let other_v_id1 =
            unwrap_or_return!(he.opposite_vertex(self), "Opposite vertex not found", false);
        let other_v_id2 = unwrap_or_return!(
            twin.opposite_vertex(self),
            "Opposite vertex not found",
            false
        );

        let pos1 = *unwrap_or_return!(self.positions.get(other_v_id1), "Position not found", false);
        let pos2 = *unwrap_or_return!(self.positions.get(other_v_id2), "Position not found", false);

        if pos1.distance_squared(pos2) > flip_threshold_sqr {
            return false;
        }

        tracing::debug!("Flipping edge {single_shared_he_id:?}");

        self.flip_edge(single_shared_he_id);

        // `collapse_edge` absorbs the edge's end vertex into its start vertex and cleans
        // up the start vertex's neighborhood, so every vertex it adds descends from it.
        let flipped_ends = self.halfedge_endpoints(single_shared_he_id);

        let CollapseEdge {
            removed_vertices,
            removed_halfedges,
            removed_faces,
            added_vertices,
        } = self.collapse_edge(single_shared_he_id);

        if let Some((start_v_id, end_v_id)) = flipped_ends {
            if removed_vertices.contains(&end_v_id) && !removed_vertices.contains(&start_v_id) {
                renames.absorb(end_v_id, start_v_id);
            }
            for &added_v_id in &added_vertices {
                renames.split(start_v_id, added_v_id);
            }
        }

        result.added_vertices.extend(added_vertices);
        result.removed_vertices.extend(removed_vertices);
        result.removed_halfedges.extend(removed_halfedges);
        result.removed_faces.extend(removed_faces);

        true
    }

    #[instrument(skip_all)]
    fn check_and_flip_single_shared_he(
        &mut self,
        shared_he_ids: &HashSet<HalfedgeId>,
        flip_threshold_sqr: f32,
        renames: &mut VertexRenames,
        result: &mut MergeVerticesOneRing,
    ) -> bool {
        let Ok(&shared_he_id) = shared_he_ids.iter().exactly_one() else {
            return false;
        };

        #[cfg(feature = "rerun")]
        self.log_he_rerun("common_one_ring_he", shared_he_id);

        self.flip_and_collapse_single_shared_edge_if_below_threshold(
            shared_he_id,
            flip_threshold_sqr,
            renames,
            result,
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConnectPairCap {
    /// Indices reference the same vertex, thus forming a closed polygon.
    Closed,
    /// Indices reference different vertices, thus forming an open polygon.
    Open,
}

/// Two ranges of elements that need to be connected by faces.
#[derive(Debug)]
struct ConnectPair {
    /// Whether the first indices in the two ranges reference the same vertex, thus forming a closed polygon.
    start_cap: ConnectPairCap,

    /// The pair of ranges of vertex indices that need to be connected.
    ranges: [RangeInclusive<usize>; 2],
}

impl ConnectPair {
    /// Connects two whole arcs of `len1` and `len2` vertices, first to last.
    fn whole(len1: usize, len2: usize, start_cap: ConnectPairCap) -> Self {
        Self::new(0..=len1 - 1, len1, 0..=len2 - 1, len2, start_cap)
    }

    fn new(
        mut range1: RangeInclusive<usize>,
        len1: usize,
        mut range2: RangeInclusive<usize>,
        len2: usize,
        start_cap: ConnectPairCap,
    ) -> Self {
        if range1.end() <= range1.start() {
            range1 = (*range1.start())..=(*range1.end() + len1);
        }
        if range2.end() <= range2.start() {
            range2 = (*range2.start())..=(*range2.end() + len2);
        }

        ConnectPair {
            start_cap,
            ranges: [range1, range2],
        }
    }

    /// The faces that connect the two ranges over two vertex rings: the Bresenham pairing
    /// ([`Self::compute_pairings`]), each pairing's fan and the quads between them.
    fn plan_faces(&self, rings: [&[VertexId]; 2]) -> Vec<PlannedFace> {
        let mut planned_faces = Vec::new();

        let pairings = self.compute_pairings();

        tracing::debug!("Pairings: {:#?}", pairings);

        let Some(last_pairing) = pairings.last() else {
            return planned_faces;
        };

        // The previous pairing's last (single, other) pair.
        let mut prev =
            (self.start_cap == ConnectPairCap::Open).then(|| last_pairing.last_pair(rings));

        for pairing in &pairings {
            if let Some((prev_single_v_id, prev_other_v_id)) = prev {
                // add the quad between the previous and current pairings
                let (single_v_id, other_v_id) = pairing.first_pair(rings);

                let face_order = if prev_single_v_id == prev_other_v_id {
                    PlannedFaceOrder::Start
                } else if single_v_id == other_v_id {
                    PlannedFaceOrder::End
                } else {
                    PlannedFaceOrder::Middle
                };

                // make sure the triangle vertices are CCW
                if pairing.single_range_idx == 0 {
                    if prev_single_v_id != prev_other_v_id {
                        planned_faces.push(PlannedFace::new(
                            prev_single_v_id,
                            single_v_id,
                            prev_other_v_id,
                            face_order,
                        ));
                    }
                    if single_v_id != other_v_id {
                        planned_faces.push(PlannedFace::new(
                            prev_other_v_id,
                            single_v_id,
                            other_v_id,
                            face_order,
                        ));
                    }
                } else {
                    if prev_single_v_id != prev_other_v_id {
                        planned_faces.push(PlannedFace::new(
                            prev_single_v_id,
                            prev_other_v_id,
                            single_v_id,
                            face_order,
                        ));
                    }
                    if single_v_id != other_v_id {
                        planned_faces.push(PlannedFace::new(
                            prev_other_v_id,
                            other_v_id,
                            single_v_id,
                            face_order,
                        ));
                    }
                }
            }

            // add the faces of the current pairing
            planned_faces.extend(pairing.fill_faces(rings));

            prev = Some(pairing.last_pair(rings));
        }

        if planned_faces.is_empty() {
            // This range pair produced no faces (e.g. a single shared vertex).
            return planned_faces;
        }

        planned_faces[0].order = PlannedFaceOrder::Start;

        let single = planned_faces.len() == 1;
        if let Some(last) = planned_faces.last_mut() {
            last.order = if single {
                PlannedFaceOrder::Single
            } else {
                PlannedFaceOrder::End
            };
        }

        planned_faces
    }

    // This is a modified Bresenham algorithm usually used for drawing lines on a grid.
    fn compute_pairings(&self) -> Vec<Pairing> {
        let range1 = self.ranges[0].clone();
        let range2 = self.ranges[1].clone();

        let count1 = range1.clone().count() as i32;
        let count2 = range2.clone().count() as i32;

        let (single_range_idx, single_count, other_count, single_range, other_range) =
            if count1 > count2 {
                (1, count2, count1, range2, range1)
            } else {
                (0, count1, count2, range1, range2)
            };

        let mut single_idx_in_range = *single_range.start();

        if single_count == 0 {
            return vec![];
        }
        if single_count == 1 {
            return vec![Pairing {
                single_range_idx,
                single_idx_in_range,
                other_range,
            }];
        }

        let mut error = 2 * single_count - other_count;

        let mut other_start = *other_range.start();
        let mut other_end = other_start;

        let mut pairings = Vec::new();

        while other_end <= *other_range.end() {
            if error > 0 {
                pairings.push(Pairing {
                    single_range_idx,
                    single_idx_in_range,
                    other_range: other_start..=other_end,
                });

                other_start = other_end + 1;
                single_idx_in_range += 1;
                error += 2 * (single_count - other_count);
            } else {
                error += 2 * single_count;
            }

            other_end += 1;
        }

        if let Some(last_pairing) = pairings.last_mut() {
            last_pairing.other_range = *last_pairing.other_range.start()..=*other_range.end();
        }

        pairings
    }
}

/// The outcome of [`MeshGraph::find_triangle_fan`].
enum TriangleFan {
    /// The halfedge spans the region between the two one rings. Holds the pairing
    /// of the fan of already existing faces around it.
    Spanning(Pairing),

    /// The halfedge doesn't span the region between the two one rings: both of its
    /// faces are intact and neither of them has its third vertex on one of the
    /// rings, so the halfedge runs along the outside of the rings instead of
    /// across the region between them. The vertices it joins must not count as
    /// connected.
    NotSpanning,

    /// The mesh connectivity around the halfedge is broken. The merge is aborted.
    Failed,
}

/// Pairs one element from one range with one or more elements from the other range.
#[derive(Debug, Clone)]
struct Pairing {
    /// The index of the range in which each single element is paired to one or more elements in the other range.
    /// Can only be `0` or `1`.
    single_range_idx: usize,

    /// The index of the single element in the range referenced by `single_range_idx`.
    single_idx_in_range: usize,

    /// The range of elements in the other range that are paired to the single element referenced in `single_idx_in_range`.
    other_range: RangeInclusive<usize>,
}

impl Pairing {
    /// `v_ids` as (the single range's ring, the other range's ring).
    fn rings<'a>(&self, v_ids: [&'a [VertexId]; 2]) -> (&'a [VertexId], &'a [VertexId]) {
        (
            v_ids[self.single_range_idx],
            v_ids[1 - self.single_range_idx],
        )
    }

    fn new_triangle(
        single_range_idx: usize,
        single_idx_in_range: usize,
        other_idx1: usize,
        other_idx2: usize,
        other_len: usize,
    ) -> Self {
        let other_range = if other_idx1 == other_idx2 {
            other_idx1..=other_idx2
        } else if other_idx1.abs_diff(other_idx2) == 1 {
            other_idx1.min(other_idx2)..=other_idx1.max(other_idx2)
        } else {
            other_idx1.max(other_idx2)..=other_idx1.min(other_idx2) + other_len
        };

        Self {
            single_range_idx,
            single_idx_in_range,
            other_range,
        }
    }

    fn all_vertex_ids(&self, v_ids: [&[VertexId]; 2]) -> Vec<VertexId> {
        let (single_ids, other_ids) = self.rings(v_ids);

        let mut all_ids = vec![];

        all_ids.push(single_ids[self.single_idx_in_range % single_ids.len()]);

        let mut idx = *self.other_range.start() % other_ids.len();
        all_ids.push(other_ids[idx]);
        while idx != *self.other_range.end() % other_ids.len() {
            idx += 1;
            idx %= other_ids.len();
            all_ids.push(other_ids[idx]);
        }

        all_ids
    }

    fn first_pair(&self, v_ids: [&[VertexId]; 2]) -> (VertexId, VertexId) {
        let (single_ids, other_ids) = self.rings(v_ids);

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let other_v_id = other_ids[*self.other_range.start() % other_ids.len()];

        (single_v_id, other_v_id)
    }

    fn last_pair(&self, v_ids: [&[VertexId]; 2]) -> (VertexId, VertexId) {
        let (single_ids, other_ids) = self.rings(v_ids);

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let other_v_id = other_ids[*self.other_range.end() % other_ids.len()];

        (single_v_id, other_v_id)
    }

    fn fill_faces(&self, v_ids: [&[VertexId]; 2]) -> Vec<PlannedFace> {
        let (single_ids, other_ids) = self.rings(v_ids);

        let single_v_id = single_ids[self.single_idx_in_range % single_ids.len()];
        let mut faces = Vec::<PlannedFace>::new();

        let mut others = self.other_range.clone();

        if others.start() == others.end() {
            return faces;
        }

        let mut prev_other_v_id = other_ids[others.next().unwrap() % other_ids.len()];
        let mut face_order = PlannedFaceOrder::Middle;

        if prev_other_v_id == single_v_id {
            face_order = PlannedFaceOrder::Start;

            if let Some(second_idx) = others.next() {
                prev_other_v_id = other_ids[second_idx % other_ids.len()];
            } else {
                return faces;
            }
        }

        for other_idx in others {
            let other_v_id = other_ids[other_idx % other_ids.len()];

            if other_v_id == single_v_id {
                break;
            }

            if other_v_id == prev_other_v_id {
                // Consecutive duplicates in the "other" ring would produce a
                // degenerate (V, S, V) self-connected face. Skip the face but
                // keep advancing so the following triangles stay well-formed.
                prev_other_v_id = other_v_id;
                continue;
            }

            // make sure the triangle vertices are CCW
            if self.single_range_idx == 0 {
                faces.push(PlannedFace::new(
                    prev_other_v_id,
                    single_v_id,
                    other_v_id,
                    face_order,
                ));
            } else {
                faces.push(PlannedFace::new(
                    prev_other_v_id,
                    other_v_id,
                    single_v_id,
                    face_order,
                ));
            }

            face_order = PlannedFaceOrder::Middle;
            prev_other_v_id = other_v_id;
        }

        faces
    }

    #[cfg(feature = "rerun")]
    fn log_rerun(
        &self,
        label: impl AsRef<str>,
        v_ids: [&[VertexId]; 2],
        mesh_graph: &MeshGraph,
    ) -> Option<()> {
        use crate::utils::vec3_array;

        let (single_v_ids, other_v_ids) = self.rings(v_ids);

        let single_v_id = single_v_ids[self.single_idx_in_range];
        let single_pos = vec3_array(mesh_graph.positions.get(single_v_id)?);

        let mut other_pos = vec![];
        let mut idx = self.other_range.start() % other_v_ids.len();

        other_pos.push(vec3_array(mesh_graph.positions.get(other_v_ids[idx])?));
        while idx != *self.other_range.end() % other_v_ids.len() {
            idx += 1;
            idx %= other_v_ids.len();
            other_pos.push(vec3_array(mesh_graph.positions.get(other_v_ids[idx])?));
        }

        crate::RR
            .log(
                format!("pairing/{}/single", label.as_ref()),
                &rerun::Points3D::new([single_pos]),
            )
            .unwrap();
        crate::RR
            .log(
                format!("pairing/{}/other", label.as_ref()),
                &rerun::LineStrips3D::new(
                    other_pos
                        .into_iter()
                        .map(|other_pos| [single_pos, other_pos]),
                ),
            )
            .unwrap();

        Some(())
    }
}

/// Represents a triangle that is planned to be added to the mesh graph.
///
/// v1 ()◀───────────────() new_he_v2
///      ╲              ▲
///       ╲            ╱
///        ╲          ╱
///         ╲        ╱
///          ╲      ╱
///           ╲    ╱
///            ▼  ╱
///             ()
///         new_he_v1
///
#[derive(Debug, Clone)]
struct PlannedFace {
    order: PlannedFaceOrder,
    v1: VertexId,
    new_he_v1: VertexId,
    new_he_v2: VertexId,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum PlannedFaceOrder {
    Start,
    Middle,
    End,
    Single,
}

impl PlannedFace {
    fn new(
        v_id1: VertexId,
        new_he_v_id1: VertexId,
        new_he_v_id2: VertexId,
        order: PlannedFaceOrder,
    ) -> Self {
        PlannedFace {
            order,
            v1: v_id1,
            new_he_v1: new_he_v_id1,
            new_he_v2: new_he_v_id2,
        }
    }

    /// Whether two of the face's vertices are the same.
    fn is_degenerate(&self) -> bool {
        self.v1 == self.new_he_v1 || self.new_he_v1 == self.new_he_v2 || self.v1 == self.new_he_v2
    }

    /// The face's three directed edges, in its winding.
    fn directed_edges(&self) -> [(VertexId, VertexId); 3] {
        [
            (self.v1, self.new_he_v1),
            (self.new_he_v1, self.new_he_v2),
            (self.new_he_v2, self.v1),
        ]
    }

    /// Pre-insertion validation, before any mutation: the three vertices must
    /// be distinct and alive. A failing face is skipped; a transaction never
    /// starts.
    fn validate_planned_face(&self, mesh_graph: &MeshGraph) -> Option<()> {
        let v_ids = [self.v1, self.new_he_v1, self.new_he_v2];
        if self.is_degenerate() {
            error!("Skipping degenerate planned face with repeated vertices: {v_ids:?}");
            return None;
        }
        for v_id in v_ids {
            if !mesh_graph.vertices.contains_key(v_id) {
                error!("Skipping planned face with dead vertex {v_id:?}: {v_ids:?}");
                return None;
            }
        }
        Some(())
    }

    #[instrument(skip(mesh_graph, record))]
    fn add_to_mesh_graph(
        &self,
        mesh_graph: &mut MeshGraph,
        record: &mut MergeRecord,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        self.log_rerun("add_to_mesh_graph", mesh_graph);

        // Transaction: validate before any mutation. Edges this face creates are
        // tracked so a failed insertion removes them again; `add_face` is the
        // commit point after which nothing can fail.
        self.validate_planned_face(mesh_graph)?;

        let mut created: Vec<HalfedgeId> = Vec::new();

        let Some(add_or_get_edge1) = mesh_graph.add_or_get_boundary_edge(self.v1, self.new_he_v1)
        else {
            // Unreachable after `validate_planned_face`.
            debug_assert!(false, "edge 1 acquisition failed after validation");
            return None;
        };
        if add_or_get_edge1.new_start_to_end {
            created.push(add_or_get_edge1.start_to_end_he_id);
        }
        let Some(add_or_get_edge2) =
            mesh_graph.add_or_get_boundary_edge(self.new_he_v1, self.new_he_v2)
        else {
            // Unreachable after `validate_planned_face`.
            debug_assert!(false, "edge 2 acquisition failed after validation");
            mesh_graph.rollback_created_edges(&mut created);
            return None;
        };
        if add_or_get_edge2.new_start_to_end {
            created.push(add_or_get_edge2.start_to_end_he_id);
        }

        // The re-filed halfedges may still belong to a live face (an overlapping
        // earlier merge's bridge or fan). Re-filing a live chain member would leave
        // the old face's chain referencing a halfedge that now claims a different
        // face (face-stealing corruption); dismantle such faces up front. They lie
        // inside the merged region and are rebuilt by the new faces.
        for he_id in [
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        ] {
            mesh_graph.dismantle_faced_member(he_id, record);
        }

        #[cfg(test)]
        mesh_graph.debug_assert_face_inputs_ready(
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        );

        let Some(mut add_face) = mesh_graph.add_face_from_halfedges(
            add_or_get_edge1.start_to_end_he_id,
            add_or_get_edge2.start_to_end_he_id,
        ) else {
            // Unreachable after the validation and readiness checks above;
            // fail loudly in tests instead of silently losing the face.
            debug_assert!(
                false,
                "planned face {self:?} failed after a clean transaction"
            );
            mesh_graph.rollback_created_edges(&mut created);
            return None;
        };

        add_face
            .halfedge_ids
            .extend(add_or_get_edge1.created_he_ids());
        add_face
            .halfedge_ids
            .extend(add_or_get_edge2.created_he_ids());

        #[cfg(test)]
        mesh_graph.debug_assert_created_edges_claimed(&created);

        let continues = matches!(
            self.order,
            PlannedFaceOrder::Start | PlannedFaceOrder::Middle
        ) && mesh_graph.halfedges[add_or_get_edge2.twin_he_id].is_boundary();

        Some((continues.then_some(add_or_get_edge2.twin_he_id), add_face))
    }

    #[instrument(skip(mesh_graph, record))]
    fn add_to_mesh_graph_and_he(
        &self,
        mesh_graph: &mut MeshGraph,
        existing_he_id: HalfedgeId,
        record: &mut MergeRecord,
    ) -> Option<(Option<HalfedgeId>, AddFace)> {
        #[cfg(feature = "rerun")]
        self.log_rerun("add_to_mesh_graph_and_he", mesh_graph);

        // Transaction: validate before any mutation. `add_face` is the commit
        // point; everything before it is rolled back on failure (created edges)
        // or cannot be undone (dismantled faces) and must therefore be
        // infallible once entered.
        self.validate_planned_face(mesh_graph)?;

        if !matches!(self.order, PlannedFaceOrder::Middle | PlannedFaceOrder::End) {
            error!("Invalid face order {:?}", self.order);
            debug_assert!(false, "planned face with invalid order reached insertion");
            return None;
        }

        // The shared edge with the previous planned face. Capture its vertex
        // pair before the dismantle: `remove_face` can free both halves (member
        // and twin) when the other side was boundary, so an id resolved here can
        // be dead afterwards. The edge is re-resolved - or re-created - by its
        // vertex pair after the dismantle.
        let (existing_start, existing_end) = mesh_graph.halfedge_endpoints(existing_he_id)?;

        let mut created: Vec<HalfedgeId> = Vec::new();

        // The edge may already exist (created by the previous planned face of this merge
        // or an overlapping earlier merge). Take that one instead of adding a second edge
        // between the same two vertices, which would leave a *parallel* halfedge pair
        // whose two free halves stay face-less forever.
        let Some(add_or_get_edge) =
            mesh_graph.add_or_get_boundary_edge(self.new_he_v1, self.new_he_v2)
        else {
            debug_assert!(
                false,
                "edge acquisition failed after `validate_planned_face`"
            );
            return None;
        };
        if add_or_get_edge.new_start_to_end {
            created.push(add_or_get_edge.start_to_end_he_id);
        }

        // A re-filed halfedge may still belong to a live face's chain.
        // Dismantle the claiming face before re-filing, and the existing
        // edge (the other side of the shared border) likewise, so no live
        // chain keeps referencing a re-filed halfedge.
        mesh_graph.dismantle_faced_member(existing_he_id, record);
        mesh_graph.dismantle_faced_member(add_or_get_edge.start_to_end_he_id, record);

        // Re-resolve the shared edge after the dismantle.
        let existing_he_id = match mesh_graph
            .boundary_half_from_to(existing_start, existing_end)
            .or_else(|| mesh_graph.boundary_half_from_to(existing_end, existing_start))
        {
            Some(he_id) => he_id,
            None => {
                // Both halves went with the dismantled face: re-create the edge.
                let Some(edge) = mesh_graph.add_or_get_boundary_edge(existing_start, existing_end)
                else {
                    // Unreachable after `validate_planned_face`.
                    debug_assert!(
                        false,
                        "re-creating the shared edge failed after a clean transaction"
                    );
                    mesh_graph.rollback_created_edges(&mut created);
                    return None;
                };
                if edge.new_start_to_end {
                    created.push(edge.start_to_end_he_id);
                }
                edge.start_to_end_he_id
            }
        };

        #[cfg(test)]
        mesh_graph
            .debug_assert_face_inputs_ready(existing_he_id, add_or_get_edge.start_to_end_he_id);

        let Some(mut add_face) =
            mesh_graph.add_face_from_halfedges(existing_he_id, add_or_get_edge.start_to_end_he_id)
        else {
            // Unreachable after the dismantle + re-resolution + readiness
            // checks above; the dismantled faces cannot be restored, so
            // fail loudly in tests rather than silently losing the face.
            debug_assert!(
                false,
                "planned face {self:?} failed after irreversible dismantles"
            );
            mesh_graph.rollback_created_edges(&mut created);
            return None;
        };

        add_face
            .halfedge_ids
            .extend(add_or_get_edge.created_he_ids());

        #[cfg(test)]
        mesh_graph.debug_assert_created_edges_claimed(&created);

        let continues = self.order == PlannedFaceOrder::Middle;
        Some((continues.then_some(add_or_get_edge.twin_he_id), add_face))
    }

    #[cfg(feature = "rerun")]
    fn log_rerun(&self, label: &str, mesh_graph: &MeshGraph) {
        use crate::{RR, utils::vec3_array};

        let (positions, labels): (Vec<_>, Vec<_>) =
            [(self.v1, "1"), (self.new_he_v1, "2"), (self.new_he_v2, "3")]
                .into_iter()
                .filter_map(|(v_id, labl)| mesh_graph.positions.get(v_id).map(|a| (a, labl)))
                .unzip();

        RR.log(
            format!("meshgraph/planned_face/{label}/positions"),
            &rerun::Points3D::new(positions.iter().map(vec3_array)).with_labels(labels),
        )
        .unwrap();

        RR.log(
            format!("meshgraph/planned_face/{label}/edges"),
            &rerun::Arrows3D::from_vectors(
                positions
                    .iter()
                    .circular_array_windows()
                    .map(|[a, b]| vec3_array(*b - *a)),
            )
            .with_origins(positions.iter().map(vec3_array)),
        )
        .unwrap();
    }
}
