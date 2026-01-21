#[cfg(feature = "rerun")]
use std::borrow::Borrow;

#[cfg(feature = "rerun")]
use glam::{Quat, Vec2, Vec3};

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
    ($msg:literal) => {
        || {
            tracing::error!($msg);
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

        let vertex_id = meshgraph.insert_vertex(point_3);
        corner_vertex_ids.push(vertex_id);
    }

    for cv_i in 0..corner_vertex_ids.len() {
        let corner_vertex_id = corner_vertex_ids[cv_i];
        let vertex_id = outer_vertex_ids[cv_i];
        let next_vertext_id = outer_vertex_ids[(cv_i + 1) % outer_vertex_ids.len()];
        let halfedge_vertex_to_corner_id = meshgraph
            .insert_or_get_edge(vertex_id, corner_vertex_id)
            .start_to_end_he_id;
        let halfedge_vertex_to_next_vertex_id = meshgraph
            .insert_or_get_edge(vertex_id, next_vertext_id)
            .start_to_end_he_id;

        meshgraph
            .create_face_from_halfedges(
                halfedge_vertex_to_corner_id,
                halfedge_vertex_to_next_vertex_id,
            )
            .unwrap();

        let halfedge_corner_to_next_vertex_id = meshgraph
            .insert_or_get_edge(corner_vertex_id, next_vertext_id)
            .start_to_end_he_id;

        let halfedge_next_vertex_to_next_corner_vertex_id = meshgraph
            .insert_or_get_edge(
                next_vertext_id,
                corner_vertex_ids[(cv_i + 1) % corner_vertex_ids.len()],
            )
            .start_to_end_he_id;

        meshgraph
            .create_face_from_halfedges(
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
    let center_id = meshgraph.insert_vertex(*center);

    let mut vertex_ids = Vec::new();
    let mut halfedge_ids = Vec::new();

    for point in points {
        let vertex_id = meshgraph.insert_vertex(*point);
        let halfedge_id = meshgraph
            .insert_or_get_edge(center_id, vertex_id)
            .start_to_end_he_id;

        vertex_ids.push(vertex_id);
        halfedge_ids.push(halfedge_id);
    }

    for i in 0..points.len() {
        meshgraph
            .create_face_from_halfedges(halfedge_ids[i], halfedge_ids[(i + 1) % points.len()])
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
