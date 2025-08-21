mod hash_grid;
mod polygon;

#[cfg(feature = "rerun")]
use std::iter::repeat_n;

use glam::{Mat4, Vec2, Vec3, Vec3Swizzles, Vec4, Vec4Swizzles};
pub use hash_grid::*;
pub use polygon::*;
use slotmap::SecondaryMap;

#[cfg(feature = "rerun")]
use crate::utils::vec3_array;
use crate::{MeshGraph, VertexId};

pub fn plane_slice(
    mesh: &MeshGraph,
    plane_normal: Vec3,
    plane_constant: f32,
) -> impl Iterator<Item = Polygon3> {
    let plane_normal = plane_normal.normalize();

    #[cfg(feature = "rerun")]
    {
        use glam::Quat;

        use crate::utils::{quat_array, vec3_array};

        crate::RR
            .log(
                "mesh",
                &rerun::Points3D::new(mesh.positions.values().map(vec3_array)),
            )
            .unwrap();

        let pivot = plane_normal * plane_constant;

        crate::RR
            .log(
                "slice_plane",
                &rerun::Ellipsoids3D::from_centers_and_half_sizes(
                    [(pivot.x, pivot.y, pivot.z)],
                    [(10.0, 10.0, 0.0)],
                )
                .with_quaternions([quat_array(Quat::from_rotation_arc_colinear(
                    Vec3::Z,
                    plane_normal,
                ))]),
            )
            .unwrap();

        crate::RR
            .log(
                "slice_plane/normal",
                &rerun::Arrows3D::from_vectors([vec3_array(plane_normal)])
                    .with_origins([vec3_array(pivot)]),
            )
            .unwrap();
    }

    let transform = compute_transform_from_plane_into_xy(plane_normal, plane_constant);

    let mut transformed_positions = SecondaryMap::new();
    let mut min_bounds = Vec2::splat(f32::INFINITY);
    let mut max_bounds = Vec2::splat(f32::NEG_INFINITY);

    for (vertex_id, vertex) in mesh.positions.iter() {
        let transformed_vertex = transform * vertex.extend(1.0);

        debug_assert!((transformed_vertex.w - 1.0).abs() < 1e-6);

        let transformed = transformed_vertex.xyz();
        let transformed_2d = transformed.xy();
        transformed_positions.insert(vertex_id, transformed);

        // Update bounding box
        min_bounds = min_bounds.min(transformed_2d);
        max_bounds = max_bounds.max(transformed_2d);
    }

    #[cfg(feature = "rerun")]
    {
        use crate::utils::vec3_array;

        crate::RR
            .log(
                "plane_slice/points_transformed",
                &rerun::Points3D::new(transformed_positions.values().copied().map(vec3_array)),
            )
            .unwrap();
    }

    let mut hash_grid = HashGrid::new(min_bounds, max_bounds);

    for face in mesh.faces.values() {
        if let Some((point1, point2)) =
            intersect_triangle_with_xy_plane(mesh, &transformed_positions, face)
        {
            hash_grid.insert_line(point1, point2);
        }
    }

    let transform_inv = transform.inverse();

    hash_grid
        .into_polygons()
        .map(move |p| Polygon3::from_polygon2_with_transform(p, transform_inv))
}

fn intersect_triangle_with_xy_plane(
    mesh: &MeshGraph,
    transformed_positions: &SecondaryMap<VertexId, Vec3>,
    face: &crate::Face,
) -> Option<(Vec2, Vec2)> {
    let vertex_ids = face.vertices(mesh);
    let vertices: Vec<Vec3> = vertex_ids
        .iter()
        .map(|&v| transformed_positions[v])
        .collect();

    // Compute signed distances from each vertex to the XY plane (z=0)
    let distances: [f32; 3] = [vertices[0].z, vertices[1].z, vertices[2].z];

    // Find intersections with triangle edges
    let mut intersection_points = Vec::new();

    for i in 0..3 {
        let j = (i + 1) % 3;
        let d1 = distances[i];
        let d2 = distances[j];

        // Check if edge crosses the XY plane (signs are different and not both zero)
        if (d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0) {
            // Compute intersection point using linear interpolation
            let t = d1 / (d1 - d2);
            let intersection = vertices[i] + t * (vertices[j] - vertices[i]);
            intersection_points.push(intersection);
        } else if d1.abs() < f32::EPSILON {
            // Vertex is exactly on the XY plane
            intersection_points.push(vertices[i]);
        }
    }

    // Remove duplicate points
    intersection_points.dedup_by(|a, b| (*a - *b).length() < f32::EPSILON);

    // Create line segment if we have exactly 2 intersections
    if intersection_points.len() == 2 {
        debug_assert!(intersection_points[0].z.abs() < 1e-6);
        debug_assert!(intersection_points[1].z.abs() < 1e-6);

        Some((intersection_points[0].xy(), intersection_points[1].xy()))
    } else {
        None
    }
}

fn compute_transform_from_plane_into_xy(plane_normal: Vec3, plane_constant: f32) -> Mat4 {
    // Create an orthonormal basis where n is the Z-axis
    // Find a vector perpendicular to n
    let up = if plane_normal.x.abs() < 0.9 {
        Vec3::X
    } else {
        Vec3::Y
    };

    // Create two perpendicular vectors in the plane
    let u = plane_normal.cross(up).normalize(); // First tangent vector
    let v = u.cross(plane_normal).normalize(); // Second tangent vector

    let point_on_plane = plane_constant * plane_normal;

    #[cfg(feature = "rerun")]
    crate::RR
        .log(
            "plane_axes",
            &rerun::Arrows3D::from_vectors([
                vec3_array(v),
                vec3_array(u),
                vec3_array(plane_normal),
            ])
            .with_origins(repeat_n(vec3_array(point_on_plane), 3))
            .with_colors([(255, 0, 0), (0, 255, 0), (0, 0, 255)]),
        )
        .unwrap();

    let rotation = Mat4::from_cols(
        v.extend(0.0),
        u.extend(0.0),
        plane_normal.extend(0.0),
        Vec4::W,
    )
    .transpose();

    let translation = Mat4::from_translation(-point_on_plane);

    rotation * translation
}

#[cfg(test)]
mod tests {
    use glam::Vec4;

    use crate::primitives::IcoSphere;

    use super::*;

    #[test]
    fn test_compute_identity_from_plane_into_xy() {
        let plane_normal = Vec3::new(0.0, 0.0, 1.0);
        let plane_constant = 0.0;

        let transform = compute_transform_from_plane_into_xy(plane_normal, plane_constant);

        assert_eq!(transform, Mat4::IDENTITY);
    }

    #[test]
    fn test_compute_transform_from_plane_into_xy() {
        let plane_normal = Vec3::new(0.0, 1.0, 1.0).normalize();
        let plane_constant = 6.3;

        let transform = compute_transform_from_plane_into_xy(plane_normal, plane_constant);

        assert_eq!(
            transform,
            Mat4 {
                x_axis: Vec4::new(1.0, 0.0, 0.0, 0.0),
                y_axis: Vec4::new(0.0, 0.7071068, 0.70710677, 0.0),
                z_axis: Vec4::new(0.0, -0.7071068, 0.70710677, 0.0),
                w_axis: Vec4::new(0.0, 0.0, -6.3, 1.0)
            }
        );
    }

    #[test]
    fn test_intersect_triangle_with_xy_plane() {
        let mesh_graph = MeshGraph::from(IcoSphere {
            radius: 2.5,
            subdivisions: 3,
        });

        let line = intersect_triangle_with_xy_plane(
            &mesh_graph,
            &mesh_graph.positions,
            mesh_graph.faces.values().next().unwrap(),
        );

        assert!(line.is_none());

        let line = intersect_triangle_with_xy_plane(
            &mesh_graph,
            &mesh_graph.positions,
            mesh_graph.faces.values().nth(64).unwrap(),
        );

        assert_eq!(
            line,
            Some((
                Vec2::new(-1.0083883, 2.2876086),
                Vec2::new(-1.3143277, 2.126627)
            ))
        );
    }

    #[test]
    fn test_icosphere_plane_slice() {
        let mesh_graph = MeshGraph::from(IcoSphere {
            radius: 2.5,
            subdivisions: 3,
        });

        let plane_normal = Vec3::new(0.0, 0.5, 1.0).normalize();
        let plane_constant = 1.2;

        let mut polygons = plane_slice(&mesh_graph, plane_normal, plane_constant);

        assert_eq!(
            polygons.next(),
            Some(Polygon3 {
                vertices: [
                    Vec3::new(2.0347278, 1.2575308, 0.71287555),
                    Vec3::new(1.9029243, 1.5103028, 0.58648956),
                    Vec3::new(1.8980179, 1.5179262, 0.58267784),
                    Vec3::new(1.8750328, 1.5487651, 0.5672584),
                    Vec3::new(1.7130562, 1.7552376, 0.46402216),
                    Vec3::new(1.6560348, 1.8131003, 0.43509078),
                    Vec3::new(1.4872973, 1.9698852, 0.35669833),
                    Vec3::new(1.3088225, 2.1019301, 0.29067588),
                    Vec3::new(1.231257, 2.1553543, 0.26396382),
                    Vec3::new(1.159522, 2.194203, 0.2445395),
                    Vec3::new(0.94288135, 2.3010523, 0.19111478),
                    Vec3::new(0.7879911, 2.3584137, 0.16243404),
                    Vec3::new(0.6283641, 2.4094386, 0.13692164),
                    Vec3::new(0.41535905, 2.455471, 0.11390537),
                    Vec3::new(0.3010591, 2.4739337, 0.10467416),
                    Vec3::new(0.05938441, 2.4911098, 0.096085966),
                    Vec3::new(-0.05938442, 2.4911098, 0.096085966),
                    Vec3::new(-0.3010591, 2.4739337, 0.10467416),
                    Vec3::new(-0.41535908, 2.455471, 0.11390537),
                    Vec3::new(-0.6283641, 2.4094386, 0.13692164),
                    Vec3::new(-0.7879911, 2.3584137, 0.16243404),
                    Vec3::new(-0.94288135, 2.3010523, 0.19111478),
                    Vec3::new(-1.159522, 2.194203, 0.2445395),
                    Vec3::new(-1.231257, 2.1553543, 0.26396382),
                    Vec3::new(-1.3088225, 2.1019301, 0.29067588),
                    Vec3::new(-1.4872973, 1.9698852, 0.35669833),
                    Vec3::new(-1.6560348, 1.8131003, 0.43509078),
                    Vec3::new(-1.7130562, 1.7552376, 0.46402216),
                    Vec3::new(-1.8750328, 1.5487651, 0.5672584),
                    Vec3::new(-1.8980179, 1.5179262, 0.58267784),
                    Vec3::new(-1.9029243, 1.5103028, 0.58648956),
                    Vec3::new(-2.0347278, 1.2575308, 0.71287555),
                    Vec3::new(-2.0585618, 1.2000002, 0.74164087),
                    Vec3::new(-2.136093, 0.96047235, 0.8614048),
                    Vec3::new(-2.1506572, 0.8914818, 0.8959),
                    Vec3::new(-2.1845584, 0.65191114, 1.0156854),
                    Vec3::new(-2.1866322, 0.5873947, 1.0479436),
                    Vec3::new(-2.1780944, 0.3281218, 1.17758),
                    Vec3::new(-2.1737561, 0.29807013, 1.1926059),
                    Vec3::new(-2.142669, 0.14740404, 1.267939),
                    Vec3::new(-2.1126394, 0.016977072, 1.3331524),
                    Vec3::new(-2.089665, -0.044861794, 1.3640718),
                    Vec3::new(-2.0094798, -0.23353845, 1.4584101),
                    Vec3::new(-1.9672765, -0.31761312, 1.5004475),
                    Vec3::new(-1.8720983, -0.46921265, 1.5762472),
                    Vec3::new(-1.7555597, -0.6294868, 1.6563843),
                    Vec3::new(-1.6925815, -0.70101833, 1.6921501),
                    Vec3::new(-1.5054793, -0.8856908, 1.7844863),
                    Vec3::new(-1.4743863, -0.9112208, 1.7972513),
                    Vec3::new(-1.1747257, -1.1182102, 1.9007461),
                    Vec3::new(-1.1614131, -1.1259825, 1.9046322),
                    Vec3::new(-0.8734361, -1.259973, 1.9716275),
                    Vec3::new(-0.8451933, -1.2710576, 1.9771698),
                    Vec3::new(-0.55759066, -1.3547609, 2.0190215),
                    Vec3::new(-0.48015964, -1.3725375, 2.0279098),
                    Vec3::new(-0.2545036, -1.4035226, 2.0434022),
                    Vec3::new(-0.09273741, -1.4171578, 2.0502198),
                    Vec3::new(0.09273741, -1.4171578, 2.0502198),
                    Vec3::new(0.2545036, -1.4035226, 2.0434022),
                    Vec3::new(0.48015964, -1.3725375, 2.0279098),
                    Vec3::new(0.55759066, -1.3547609, 2.0190215),
                    Vec3::new(0.8451933, -1.2710576, 1.9771698),
                    Vec3::new(0.8734361, -1.259973, 1.9716275),
                    Vec3::new(1.1614131, -1.1259825, 1.9046322),
                    Vec3::new(1.1747257, -1.1182102, 1.9007461),
                    Vec3::new(1.4743863, -0.9112208, 1.7972513),
                    Vec3::new(1.5054793, -0.8856908, 1.7844863),
                    Vec3::new(1.6925815, -0.70101833, 1.6921501),
                    Vec3::new(1.7555597, -0.6294868, 1.6563843),
                    Vec3::new(1.8720983, -0.46921265, 1.5762472),
                    Vec3::new(1.9672765, -0.31761312, 1.5004475),
                    Vec3::new(2.0094798, -0.23353845, 1.4584101),
                    Vec3::new(2.089665, -0.044861794, 1.3640718),
                    Vec3::new(2.1126394, 0.016977072, 1.3331524),
                    Vec3::new(2.142669, 0.14740404, 1.267939),
                    Vec3::new(2.1737561, 0.29807013, 1.1926059),
                    Vec3::new(2.1780944, 0.3281218, 1.17758),
                    Vec3::new(2.1866322, 0.5873947, 1.0479436),
                    Vec3::new(2.1845584, 0.65191114, 1.0156854),
                    Vec3::new(2.1506572, 0.89148176, 0.8959001),
                    Vec3::new(2.136093, 0.96047235, 0.8614048),
                    Vec3::new(2.0585618, 1.2000002, 0.74164087),
                    Vec3::new(2.0347278, 1.2575308, 0.71287555)
                ]
                .into()
            })
        );

        assert!(polygons.next().is_none());
    }
}
