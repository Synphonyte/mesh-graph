//! Tests for the selection-scoped variants of the edge-length cleanups.
//!
//! These cover three separate promises, which fail independently:
//!
//! 1. **Bookkeeping** - the selection handed back names live elements, and names every
//!    element the operation created. Asserted against a brute-force diff of the mesh's
//!    own keys, so the mechanism the operation uses to report additions is never the
//!    thing being trusted.
//! 2. **Containment** - the operation changes geometry inside the selection and leaves
//!    the rest of the mesh alone.
//! 3. **Equivalence** - selecting everything reproduces the whole-mesh operation.

use glam::Vec3;
use hashbrown::HashSet;

use crate::{
    EdgeLengthCleanup, FaceId, HalfedgeId, MeshGraph, ScopedCleanup, Selection, SelectionOps,
    VertexId,
    utils::{
        build_grid, edge_endpoint_positions, element_keys, mesh_invariant_violations,
        select_faces_where, selection_area, selection_violations,
    },
};

type Keys = (HashSet<VertexId>, HashSet<HalfedgeId>, HashSet<FaceId>);

/// Asserts the full bookkeeping contract against a brute-force key diff.
///
/// The contract, per element kind:
/// ```text
/// selection_after == (selection_before ∩ live_after) ∪ (created_and_still_live)
/// ```
/// plus: the reported delta is sufficient to get from one to the other, nothing is in
/// both halves of the delta, and no dead id survives anywhere.
fn assert_bookkeeping(
    mg: &MeshGraph,
    before: &Keys,
    sel_before: &Selection,
    sel_after: &Selection,
    result: &ScopedCleanup,
) {
    let (live_v, live_he, live_f) = element_keys(mg);

    // No dead ids anywhere in the returned selection.
    assert!(
        selection_violations(mg, sel_after).is_empty(),
        "selection holds dead ids: {:?}",
        selection_violations(mg, sel_after)
    );

    // The set equation, kind by kind.
    let expect_v: HashSet<VertexId> = sel_before
        .vertices
        .iter()
        .copied()
        .filter(|id| live_v.contains(id))
        .chain(live_v.difference(&before.0).copied())
        .collect();
    assert_eq!(sel_after.vertices, expect_v, "vertex bookkeeping");

    let expect_he: HashSet<HalfedgeId> = sel_before
        .halfedges
        .iter()
        .copied()
        .filter(|id| live_he.contains(id))
        .chain(live_he.difference(&before.1).copied())
        .collect();
    assert_eq!(sel_after.halfedges, expect_he, "halfedge bookkeeping");

    let expect_f: HashSet<FaceId> = sel_before
        .faces
        .iter()
        .copied()
        .filter(|id| live_f.contains(id))
        .chain(live_f.difference(&before.2).copied())
        .collect();
    assert_eq!(sel_after.faces, expect_f, "face bookkeeping");

    // Each created element must be reported once. Comparing as sets below would hide a
    // double-recorded id, which would make a caller replaying the delta insert twice.
    assert_eq!(
        result.added.vertices.len(),
        result.added.vertices.iter().collect::<HashSet<_>>().len(),
        "duplicate vertex in the added list"
    );
    assert_eq!(
        result.added.halfedges.len(),
        result.added.halfedges.iter().collect::<HashSet<_>>().len(),
        "duplicate halfedge in the added list"
    );
    assert_eq!(
        result.added.faces.len(),
        result.added.faces.iter().collect::<HashSet<_>>().len(),
        "duplicate face in the added list"
    );

    // `added` must be exactly what is newly live, and all of it alive.
    assert_eq!(
        result.added.vertices.iter().copied().collect::<HashSet<_>>(),
        live_v.difference(&before.0).copied().collect::<HashSet<_>>(),
        "reported added vertices"
    );
    assert_eq!(
        result
            .added
            .halfedges
            .iter()
            .copied()
            .collect::<HashSet<_>>(),
        live_he.difference(&before.1).copied().collect::<HashSet<_>>(),
        "reported added halfedges"
    );
    assert_eq!(
        result.added.faces.iter().copied().collect::<HashSet<_>>(),
        live_f.difference(&before.2).copied().collect::<HashSet<_>>(),
        "reported added faces"
    );

    // `removed` must be dead, and must have been selected beforehand.
    for v_id in &result.removed.vertices {
        assert!(!live_v.contains(v_id), "removed vertex {v_id:?} is still live");
        assert!(
            sel_before.vertices.contains(v_id),
            "removed vertex {v_id:?} was never selected"
        );
    }
    for he_id in &result.removed.halfedges {
        assert!(
            !live_he.contains(he_id),
            "removed halfedge {he_id:?} is still live"
        );
    }
    for f_id in &result.removed.faces {
        assert!(!live_f.contains(f_id), "removed face {f_id:?} is still live");
    }

    // Nothing may be reported as both added and removed - that is the ambiguity a
    // remove-journal would introduce and which deriving removals from liveness avoids.
    let added_v: HashSet<_> = result.added.vertices.iter().copied().collect();
    let removed_v: HashSet<_> = result.removed.vertices.iter().copied().collect();
    assert!(
        added_v.is_disjoint(&removed_v),
        "a vertex was reported as both added and removed"
    );

    // Delta sufficiency: replaying the delta onto the old selection rebuilds the new one.
    let mut replayed = sel_before.clone();
    replayed.remove_edit(&result.removed);
    replayed.extend_with_edit(&result.added);
    assert_eq!(replayed.vertices, sel_after.vertices, "replayed vertices");
    assert_eq!(replayed.halfedges, sel_after.halfedges, "replayed halfedges");
    assert_eq!(replayed.faces, sel_after.faces, "replayed faces");
}

/// The vertices the scope covers: the endpoints of every halfedge the selection resolves
/// to. Collapse may move these; everything else must hold still.
fn scope_vertices(mg: &MeshGraph, sel: &Selection) -> HashSet<VertexId> {
    let mut verts = HashSet::new();

    for he_id in sel.resolve_to_halfedges(mg) {
        let Some(he) = mg.halfedges.get(he_id) else {
            continue;
        };
        if let Some(start) = he.start_vertex(mg) {
            verts.insert(start);
        }
        verts.insert(he.end_vertex);
    }

    verts
}

/// The scope exactly as `run_scoped` materialises it: both directions of every edge
/// the selection resolves to. Must be captured *before* the operation — subdivision
/// re-pairs twins, so deriving it afterwards names a different set.
fn full_scope(mg: &MeshGraph, sel: &Selection) -> HashSet<HalfedgeId> {
    let mut scope = HashSet::new();

    for he_id in sel.resolve_to_halfedges(mg) {
        scope.insert(he_id);
        if let Some(twin_id) = mg.halfedges.get(he_id).and_then(|he| he.twin) {
            scope.insert(twin_id);
        }
    }

    scope
}

/// Total area of every live face. Subdivision must not change it.
fn mesh_area(mg: &MeshGraph) -> f32 {
    mg.faces
        .keys()
        .filter_map(|f_id| mg.faces.get(f_id))
        .map(|face| {
            let p: Vec<Vec3> = face.vertex_positions(mg).collect();
            (p[1] - p[0]).cross(p[2] - p[0]).length() * 0.5
        })
        .sum()
}

fn left_half(mg: &MeshGraph) -> Selection {
    select_faces_where(mg, |c| c.x < 2.0)
}

fn sorted_positions(mg: &MeshGraph) -> Vec<Vec3> {
    let mut ps: Vec<Vec3> = mg.positions.values().copied().collect();
    ps.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then(a.y.total_cmp(&b.y))
            .then(a.z.total_cmp(&b.z))
    });
    ps
}

// ---------------------------------------------------------------- A. bookkeeping

#[test]
fn test_scoped_subdivide_selection_matches_brute_force_key_diff() {
    for (label, sel) in selection_shapes() {
        let mut mg = build_grid(4);
        let sel = &mut sel(&mg);
        let before = element_keys(&mg);
        let sel_before = sel.clone();

        let result = mg.subdivide_selected_until_edges_below_max_length(
            sel,
            0.3,
            &mut HashSet::new(),
            &mut HashSet::new(),
        );

        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "[{label}] mesh broken: {:?}",
            mesh_invariant_violations(&mg)
        );
        assert_bookkeeping(&mg, &before, &sel_before, sel, &result);
    }
}

#[test]
fn test_scoped_collapse_selection_matches_brute_force_key_diff() {
    for (label, sel) in selection_shapes() {
        let mut mg = build_grid(4);
        let sel = &mut sel(&mg);
        let before = element_keys(&mg);
        let sel_before = sel.clone();

        // 1.5 sits between the grid's axis edges (1.0) and its diagonals (2.0), so every
        // axis edge in scope is a collapse candidate.
        let result =
            mg.collapse_selected_until_edges_above_min_length(sel, 1.5, &mut HashSet::new());

        assert!(
            mesh_invariant_violations(&mg).is_empty(),
            "[{label}] mesh broken: {:?}",
            mesh_invariant_violations(&mg)
        );
        assert_bookkeeping(&mg, &before, &sel_before, sel, &result);
    }
}

/// The three shapes a `Selection` can take, which `resolve_to_halfedges` treats
/// differently and which the write-back must handle alike.
#[allow(clippy::type_complexity)]
fn selection_shapes() -> Vec<(&'static str, Box<dyn Fn(&MeshGraph) -> Selection>)> {
    vec![
        ("faces", Box::new(left_half)),
        (
            "halfedges",
            Box::new(|mg: &MeshGraph| {
                let sel = left_half(mg);
                Selection::from_iter(sel.resolve_to_halfedges(mg))
            }),
        ),
        (
            "vertices",
            Box::new(|mg: &MeshGraph| {
                let sel = left_half(mg);
                Selection::from_iter(sel.resolve_to_vertices(mg))
            }),
        ),
    ]
}

#[test]
fn test_scoped_ops_leave_no_stale_ids() {
    let mut mg = build_grid(5);
    let mut sel = left_half(&mg);

    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.2,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );
    assert!(selection_violations(&mg, &sel).is_empty());
    assert!(mesh_invariant_violations(&mg).is_empty());

    mg.collapse_selected_until_edges_above_min_length(&mut sel, 0.5, &mut HashSet::new());
    assert!(
        selection_violations(&mg, &sel).is_empty(),
        "{:?}",
        selection_violations(&mg, &sel)
    );
    assert!(
        mesh_invariant_violations(&mg).is_empty(),
        "{:?}",
        mesh_invariant_violations(&mg)
    );
}

/// An element created and then destroyed inside one run must appear in neither list.
///
/// Deriving removals from liveness is what makes this hold: such an element is not live
/// at the end, so it never enters `added`, and it was never in the selection, so it
/// never enters `removed`. Two separate journals would have to cancel them explicitly.
#[test]
fn test_element_created_and_destroyed_within_one_op_appears_in_neither_list() {
    let mut mg = build_grid(4);
    let mut sel = Selection::select_all(&mg);

    // Refine hard, then collapse hard: the collapse drives `make_vertex_neighborhood_manifold`,
    // which can split a vertex and later remove it again.
    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.2,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    let before = element_keys(&mg);
    let sel_before = sel.clone();
    let result = mg.collapse_selected_until_edges_above_min_length(&mut sel, 1.0, &mut HashSet::new());

    assert_bookkeeping(&mg, &before, &sel_before, &sel, &result);
    assert!(mesh_invariant_violations(&mg).is_empty());
}

// ---------------------------------------------------------------- B. containment

/// Splitting an edge re-points its `end_vertex` at the new center, so an out-of-scope
/// edge whose endpoint positions are unchanged is an edge that was never split.
#[test]
fn test_scoped_subdivide_never_splits_an_out_of_scope_edge() {
    let mut mg = build_grid(4);
    let mut sel = left_half(&mg);

    let scope = full_scope(&mg, &sel);
    let before_positions = edge_endpoint_positions(&mg);

    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.3,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    let after_positions = edge_endpoint_positions(&mg);
    let mut checked = 0;

    for (he_id, before) in &before_positions {
        if scope.contains(he_id) {
            continue;
        }

        checked += 1;
        assert_eq!(
            after_positions.get(he_id),
            Some(before),
            "out-of-scope edge {he_id:?} was split or moved"
        );
    }

    assert!(checked > 0, "fixture put every edge in scope - test is vacuous");
}

/// Collapse moves exactly one vertex per edge - the surviving endpoint, always an
/// endpoint of a scope edge. Everything outside must hold still.
#[test]
fn test_scoped_collapse_moves_no_vertex_outside_the_scope() {
    let mut mg = build_grid(5);
    let mut sel = left_half(&mg);

    let in_scope = scope_vertices(&mg, &sel);
    let before: Vec<(VertexId, Vec3)> = mg.positions.iter().map(|(v, &p)| (v, p)).collect();

    mg.collapse_selected_until_edges_above_min_length(&mut sel, 1.5, &mut HashSet::new());

    let mut checked = 0;
    for (v_id, pos) in before {
        if in_scope.contains(&v_id) {
            continue;
        }
        let Some(now) = mg.positions.get(v_id) else {
            panic!("vertex {v_id:?} outside the scope was removed");
        };
        checked += 1;
        assert_eq!(*now, pos, "vertex {v_id:?} outside the scope moved");
    }

    assert!(checked > 0, "fixture put every vertex in scope - test is vacuous");
    assert!(mesh_invariant_violations(&mg).is_empty());
}

#[test]
fn test_empty_selection_is_a_no_op() {
    let mut mg = build_grid(3);
    let before = element_keys(&mg);
    let mut sel = Selection::default();

    let sub = mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.1,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );
    let col = mg.collapse_selected_until_edges_above_min_length(&mut sel, 10.0, &mut HashSet::new());

    assert!(sub.converged(), "an empty selection has nothing left to fix");
    assert!(col.converged());
    assert!(sub.added.is_empty() && sub.removed.is_empty());
    assert!(col.added.is_empty() && col.removed.is_empty());
    assert_eq!(element_keys(&mg), before, "the mesh was modified");
    assert!(sel.vertices.is_empty() && sel.halfedges.is_empty() && sel.faces.is_empty());
}

// ---------------------------------------------------------------- C. coverage

/// Subdivision splits faces without changing the area they cover, so the region the
/// selection describes must come through unchanged.
///
/// Measured on the originally selected region alone (`x < 2.0`): every descendant of an
/// originally selected face lies inside its parent, so the area there is exactly
/// preserved. A created face missing from the selection loses area, and a dead face left
/// in it is skipped by `selection_area` and loses area too.
///
/// The selection *as a whole* grows past that, and is supposed to: splitting an edge on
/// the rim also splits the unselected face on the far side, and every created element
/// joins the selection. That surplus is asserted separately so the two effects cannot
/// mask each other.
#[test]
fn test_scoped_subdivide_preserves_selected_area() {
    let mut mg = build_grid(4);
    let mut sel = left_half(&mg);

    let region_before = selection_area(&mg, &sel);
    let mesh_before = mesh_area(&mg);
    assert!(region_before > 0.0);

    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.3,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    // Subdivision never changes the area of the mesh.
    assert!(
        (mesh_area(&mg) - mesh_before).abs() < 1e-3,
        "subdivision changed the mesh area"
    );

    let mut region_after = 0.0;
    let mut far_side_after = 0.0;
    for f_id in &sel.faces {
        let Some(face) = mg.faces.get(*f_id) else {
            continue;
        };
        let p: Vec<Vec3> = face.vertex_positions(&mg).collect();
        let area = (p[1] - p[0]).cross(p[2] - p[0]).length() * 0.5;
        if face.center(&mg).x < 2.0 {
            region_after += area;
        } else {
            far_side_after += area;
        }
    }

    assert!(
        (region_after - region_before).abs() < 1e-3,
        "selected area inside the original region went from {region_before} to {region_after} - a created face was dropped, or a dead one kept"
    );
    assert!(
        far_side_after > 0.0,
        "expected rim splits to pull far-side faces into the selection"
    );
}

/// A selection naming one edge must name both of its halves afterwards.
///
/// `subdivide_edge` re-pairs twins, so the original halfedge ends up covering only the
/// first half. An implementation that canonicalises to `min(he, twin)` and never revisits
/// it keeps one half and silently drops the other.
#[test]
fn test_scoped_subdivide_selects_both_halves_of_a_split_edge() {
    let mut mg = build_grid(2);

    // A single diagonal (length² 2.0), the longest edge class in the grid.
    let (he_id, start_pos, end_pos) = mg
        .halfedges
        .iter()
        .find_map(|(he_id, he)| {
            let start = he.start_vertex(&mg)?;
            let a = *mg.positions.get(start)?;
            let b = *mg.positions.get(he.end_vertex)?;
            ((b - a).length_squared() > 1.5).then_some((he_id, a, b))
        })
        .expect("grid has diagonals");

    let mut sel = Selection::default();
    sel.insert(he_id);

    // Between the halves (0.5) and the whole diagonal (2.0): exactly one split.
    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        1.0,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    let mid = (start_pos + end_pos) * 0.5;
    let covers = |a: Vec3, b: Vec3| {
        sel.halfedges.iter().any(|he_id| {
            let Some(he) = mg.halfedges.get(*he_id) else {
                return false;
            };
            let Some(start) = he.start_vertex(&mg) else {
                return false;
            };
            let (Some(p), Some(q)) = (mg.positions.get(start), mg.positions.get(he.end_vertex))
            else {
                return false;
            };
            (p.abs_diff_eq(a, 1e-5) && q.abs_diff_eq(b, 1e-5))
                || (p.abs_diff_eq(b, 1e-5) && q.abs_diff_eq(a, 1e-5))
        })
    };

    assert!(covers(start_pos, mid), "first half of the split edge is not selected");
    assert!(covers(mid, end_pos), "second half of the split edge is not selected");
}

// ---------------------------------------------------------------- D. equivalence

#[test]
fn test_select_all_scoped_matches_unscoped_subdivide() {
    let mut unscoped = build_grid(4);
    let mut scoped = unscoped.clone();

    unscoped.subdivide_until_edges_below_max_length(0.3, &mut HashSet::new(), &mut HashSet::new());

    let mut sel = Selection::select_all(&scoped);
    scoped.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.3,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    assert_eq!(scoped.vertices.len(), unscoped.vertices.len());
    assert_eq!(scoped.halfedges.len(), unscoped.halfedges.len());
    assert_eq!(scoped.faces.len(), unscoped.faces.len());
    assert_eq!(sorted_positions(&scoped), sorted_positions(&unscoped));
}

#[test]
fn test_select_all_scoped_matches_unscoped_collapse() {
    let mut unscoped = build_grid(4);
    let mut scoped = unscoped.clone();

    unscoped.collapse_until_edges_above_min_length(1.5, &mut HashSet::new());

    let mut sel = Selection::select_all(&scoped);
    scoped.collapse_selected_until_edges_above_min_length(&mut sel, 1.5, &mut HashSet::new());

    assert_eq!(scoped.vertices.len(), unscoped.vertices.len());
    assert_eq!(scoped.halfedges.len(), unscoped.halfedges.len());
    assert_eq!(scoped.faces.len(), unscoped.faces.len());
    assert_eq!(sorted_positions(&scoped), sorted_positions(&unscoped));
}

/// Idempotence is asserted over `select_all`, the one selection the scope creep cannot
/// widen: it already covers the mesh. A partial selection legitimately grows between
/// calls — see `test_scoped_subdivide_scope_creeps_by_design` — so re-running on one
/// tests the creep, not the operation.
#[test]
fn test_scoped_subdivide_is_idempotent_once_converged() {
    let mut mg = build_grid(4);
    let mut sel = Selection::select_all(&mg);

    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.9,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    let keys = element_keys(&mg);
    let sel_snapshot = sel.clone();

    let again = mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.9,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    assert!(again.converged());
    assert_eq!(element_keys(&mg), keys, "second call changed the mesh");
    assert!(again.added.is_empty(), "second call reported additions");
    assert!(again.removed.is_empty(), "second call reported removals");
    assert_eq!(sel.vertices, sel_snapshot.vertices);
    assert_eq!(sel.halfedges, sel_snapshot.halfedges);
    assert_eq!(sel.faces, sel_snapshot.faces);
}

// ---------------------------------------------------------------- E. robustness

/// A selection invalidated by an unrelated operation must degrade, not panic.
///
/// `resolve_to_halfedges` used to index the slotmaps directly, so this was a panic.
#[test]
fn test_scoped_op_tolerates_a_stale_selection() {
    let mut mg = build_grid(4);
    let mut sel = Selection::select_all(&mg);
    sel.vertices = mg.vertices.keys().collect();
    sel.halfedges = mg.halfedges.keys().collect();

    // Invalidate it behind the selection's back.
    mg.collapse_until_edges_above_min_length(1.5, &mut HashSet::new());
    assert!(
        !selection_violations(&mg, &sel).is_empty(),
        "fixture failed to invalidate anything - test is vacuous"
    );

    let sel_before = sel.clone();
    let before = element_keys(&mg);
    let result = mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.5,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    assert!(selection_violations(&mg, &sel).is_empty());
    assert!(mesh_invariant_violations(&mg).is_empty());
    // Ids that were already dead on entry are reported too, so the caller's own mirror
    // of the selection can be brought back in step.
    assert!(!result.removed.is_empty());
    assert_bookkeeping(&mg, &before, &sel_before, &sel, &result);
}

/// Pins the documented consequence of adding created elements to all three sets: a
/// face-based selection accumulates vertices, and a vertex resolves its whole one-ring,
/// so a later call sees a wider scope. Documented on both methods; asserted here so a
/// change in the behaviour is a failing test rather than a surprise.
#[test]
fn test_scoped_subdivide_scope_creeps_by_design() {
    let mut mg = build_grid(4);
    let mut sel = left_half(&mg);

    let scope_before: HashSet<HalfedgeId> = sel.resolve_to_halfedges(&mg);

    mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.3,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    let scope_after: HashSet<HalfedgeId> = sel.resolve_to_halfedges(&mg);
    let live_before: HashSet<HalfedgeId> = scope_before
        .into_iter()
        .filter(|id| mg.halfedges.contains_key(*id))
        .collect();

    assert!(
        live_before.is_subset(&scope_after),
        "the scope lost edges it used to cover"
    );
    assert!(
        scope_after.len() > live_before.len(),
        "expected the scope to widen"
    );
}

/// The outcome is still reported, and still means what it does for the whole-mesh ops.
#[test]
fn test_scoped_subdivide_reports_converged_when_it_drains() {
    let mut mg = build_grid(4);
    let mut sel = left_half(&mg);
    let scope = full_scope(&mg, &sel);

    let result = mg.subdivide_selected_until_edges_below_max_length(
        &mut sel,
        0.9,
        &mut HashSet::new(),
        &mut HashSet::new(),
    );

    assert_eq!(result.outcome, EdgeLengthCleanup::Converged);
    assert!(result.converged());

    // Only the edges that were in scope when the call started: the selection is wider
    // afterwards by design, and those extra edges were never this call's to fix.
    for he_id in scope {
        let Some(he) = mg.halfedges.get(he_id) else {
            continue;
        };
        assert!(
            he.length_squared(&mg) <= 0.9 + 1e-6,
            "edge {he_id:?} in scope is still too long"
        );
    }
}

