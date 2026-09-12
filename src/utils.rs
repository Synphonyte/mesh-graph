#[cfg(feature = "rerun")]
use std::borrow::Borrow;

#[cfg(any(test, feature = "rerun"))]
use glam::Vec3;
#[cfg(feature = "rerun")]
use glam::{Quat, Vec2};

#[cfg(feature = "rerun")]
pub fn vec3_array(v: impl Borrow<Vec3>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, v.borrow().z]
}

#[cfg(feature = "rerun")]
pub fn vec2_array(v: impl Borrow<Vec2>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, 0.0]
}

#[cfg(feature = "rerun")]
pub fn quat_array(q: impl Borrow<Quat>) -> [f32; 4] {
    [q.borrow().x, q.borrow().y, q.borrow().z, q.borrow().w]
}

#[macro_export]
macro_rules! error_none {
    ($msg:literal $(, $args:expr)*) => {
        || {
            tracing::error!($msg $(, $args)*);
            None
        }
    };
}

macro_rules! unwrap_or_return {
    ($code:expr, $error:expr, $ret:expr) => {
        match $code {
            Some(value) => value,
            None => {
                tracing::error!($error);
                return $ret;
            }
        }
    };
    ($code:expr, $error:expr) => {
        match $code {
            Some(value) => value,
            None => {
                tracing::error!($error);
                return;
            }
        }
    };
}

// Shared test helpers. Used by the `ops::collapse` and `ops::subdivide` test
// modules, which both need a mesh with a known edge-length distribution and a
// way to assert the half-edge invariants survived an operation.
#[cfg(test)]
use crate::{HalfedgeId, MeshGraph, VertexId};

/// Builds an `(n x n)` grid of quads in the XY plane, each quad split into two
/// triangles (CCW when viewed from `+z`), from scratch with the public add APIs.
#[cfg(test)]
pub(crate) fn build_grid(n: usize) -> MeshGraph {
    let mut g = MeshGraph::new();

    let cell = 1.0;
    let mut v = vec![vec![]; n + 1];
    for (j, row) in v.iter_mut().enumerate() {
        for i in 0..=n {
            let id = g.add_vertex(Vec3::new(i as f32 * cell, j as f32 * cell, 0.0));
            row.push(id);
        }
    }

    let edge = |g: &mut MeshGraph, a: VertexId, b: VertexId| -> HalfedgeId {
        g.add_or_get_edge(a, b).unwrap().start_to_end_he_id
    };

    for j in 0..n {
        for i in 0..n {
            let a = v[j][i];
            let b = v[j][i + 1];
            let c = v[j + 1][i + 1];
            let d = v[j + 1][i];

            // triangles a-b-c and a-c-d
            let he_ab = edge(&mut g, a, b);
            let he_bc = edge(&mut g, b, c);
            let he_ca = edge(&mut g, c, a);
            g.add_face(he_ab, he_bc, he_ca);

            let he_ac = edge(&mut g, a, c);
            let he_cd = edge(&mut g, c, d);
            let he_da = edge(&mut g, d, a);
            g.add_face(he_ac, he_cd, he_da);
        }
    }

    g
}

/// Returns a human-readable list of broken half-edge invariants, empty when the
/// mesh is well formed: mutual twins, live start/end vertices with positions,
/// live `face`/`next`, and `outgoing_halfedges` agreeing with the topology.
#[cfg(test)]
pub(crate) fn mesh_invariant_violations(mg: &MeshGraph) -> Vec<String> {
    let mut problems = Vec::new();

    for (he_id, he) in &mg.halfedges {
        let Some(twin_id) = he.twin else {
            problems.push(format!("he {he_id:?}: missing twin"));
            continue;
        };
        match mg.halfedges.get(twin_id) {
            Some(twin) if twin.twin == Some(he_id) => {}
            _ => problems.push(format!(
                "he {he_id:?}: twin {twin_id:?} does not point back"
            )),
        }

        let Some(sv) = he.start_vertex(mg) else {
            problems.push(format!("he {he_id:?}: no start vertex"));
            continue;
        };
        if !mg.vertices.contains_key(sv) || !mg.positions.contains_key(sv) {
            problems.push(format!("he {he_id:?}: start vertex {sv:?} is dead"));
        }
        if !mg.vertices.contains_key(he.end_vertex) || !mg.positions.contains_key(he.end_vertex) {
            problems.push(format!(
                "he {he_id:?}: end vertex {:?} is dead",
                he.end_vertex
            ));
        }
        if let Some(f) = he.face
            && !mg.faces.contains_key(f)
        {
            problems.push(format!("he {he_id:?}: face {f:?} is dead"));
        }
        if let Some(n) = he.next
            && !mg.halfedges.contains_key(n)
        {
            problems.push(format!("he {he_id:?}: next {n:?} is dead"));
        }

        // halfedge must be present in its start vertex's outgoing list
        match mg.outgoing_halfedges.get(sv) {
            Some(list) if list.contains(&he_id) => {}
            _ => problems.push(format!("he {he_id:?}: not in outgoing_halfedges[{sv:?}]")),
        }
    }

    for (v_id, list) in &mg.outgoing_halfedges {
        for &he_id in list {
            let Some(he) = mg.halfedges.get(he_id) else {
                problems.push(format!("outgoing_halfedges[{v_id:?}]: stale he {he_id:?}"));
                continue;
            };
            if he.start_vertex(mg) != Some(v_id) {
                problems.push(format!(
                    "outgoing_halfedges[{v_id:?}]: he {he_id:?} does not start here"
                ));
            }
        }
    }

    problems
}

#[cfg(test)]
pub(crate) fn get_tracing_subscriber() {
    if let Err(e) = tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_line_number(true)
        .pretty()
        .try_init()
    {
        tracing::warn!("Tracing subscriber already initialized: {}", e);
    }
}

pub(crate) use unwrap_or_return;

#[cfg(test)]
fn extend_outer_corners(
    meshgraph: &mut crate::MeshGraph,
    new_vertex_ids: &mut Vec<crate::VertexId>,
    outer_vertex_ids: &[crate::VertexId],
    scalar: f32,
    steps: usize,
) {
    if steps == 0 {
        return;
    }

    let mut corner_vertex_ids = Vec::with_capacity(outer_vertex_ids.len());

    // mesh star corners to make the mesh larger
    for i in 0..outer_vertex_ids.len() {
        let point_1 = meshgraph.positions.get(outer_vertex_ids[i]).unwrap();
        let point_2 = meshgraph
            .positions
            .get(outer_vertex_ids[(i + 1) % outer_vertex_ids.len()])
            .unwrap();

        let mut point_3 = point_1
            + ((point_2 - point_1) * 0.5)
            + (point_1 + point_2).normalize() * scalar / steps as f32;

        point_3.z = 0.0; // allow expansion only in x-y-plane

        let vertex_id = meshgraph.add_vertex(point_3);
        corner_vertex_ids.push(vertex_id);
    }

    for cv_i in 0..corner_vertex_ids.len() {
        let corner_vertex_id = corner_vertex_ids[cv_i];
        let vertex_id = outer_vertex_ids[cv_i];
        let next_vertext_id = outer_vertex_ids[(cv_i + 1) % outer_vertex_ids.len()];
        let halfedge_vertex_to_corner_id = meshgraph
            .add_or_get_edge(vertex_id, corner_vertex_id)
            .unwrap()
            .start_to_end_he_id;
        let halfedge_vertex_to_next_vertex_id = meshgraph
            .add_or_get_edge(vertex_id, next_vertext_id)
            .unwrap()
            .start_to_end_he_id;

        meshgraph
            .add_face_from_halfedges(
                halfedge_vertex_to_corner_id,
                halfedge_vertex_to_next_vertex_id,
            )
            .unwrap();

        let halfedge_corner_to_next_vertex_id = meshgraph
            .add_or_get_edge(corner_vertex_id, next_vertext_id)
            .unwrap()
            .start_to_end_he_id;

        let halfedge_next_vertex_to_next_corner_vertex_id = meshgraph
            .add_or_get_edge(
                next_vertext_id,
                corner_vertex_ids[(cv_i + 1) % corner_vertex_ids.len()],
            )
            .unwrap()
            .start_to_end_he_id;

        meshgraph
            .add_face_from_halfedges(
                halfedge_corner_to_next_vertex_id,
                halfedge_next_vertex_to_next_corner_vertex_id,
            )
            .unwrap();
    }

    extend_outer_corners(
        meshgraph,
        new_vertex_ids,
        &corner_vertex_ids,
        scalar,
        steps - 1,
    );

    new_vertex_ids.extend(corner_vertex_ids);
}

/// Extend a mesh graph with new points.
/// Expects the first point to be the geometrical center of the new vertices.
/// Mesh then extends further by `steps` iterations from the center outward.
#[cfg(test)]
pub(crate) fn extend_with(
    meshgraph: &mut crate::MeshGraph,
    center_and_points: &[Vec3],
    matrix: glam::Mat4,
    scalar: f32,
    steps: usize,
) -> Vec<crate::VertexId> {
    let (center, points) = center_and_points.split_first().unwrap();
    let center_id = meshgraph.add_vertex(*center);

    let mut vertex_ids = Vec::new();
    let mut halfedge_ids = Vec::new();

    for point in points {
        let vertex_id = meshgraph.add_vertex(*point);
        let halfedge_id = meshgraph
            .add_or_get_edge(center_id, vertex_id)
            .unwrap()
            .start_to_end_he_id;

        vertex_ids.push(vertex_id);
        halfedge_ids.push(halfedge_id);
    }

    for i in 0..points.len() {
        meshgraph
            .add_face_from_halfedges(halfedge_ids[i], halfedge_ids[(i + 1) % points.len()])
            .unwrap();
    }

    let mut new_vertex_ids = vec![center_id];
    new_vertex_ids.extend(vertex_ids.clone());
    extend_outer_corners(meshgraph, &mut new_vertex_ids, &vertex_ids, scalar, steps);

    for new_vertex_id in new_vertex_ids.iter() {
        if let Some(pos) = meshgraph.positions.get_mut(*new_vertex_id) {
            *pos = matrix.transform_point3(*pos);
        };
    }

    new_vertex_ids
}
