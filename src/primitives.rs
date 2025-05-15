use glam::{Vec3, vec3};
use itertools::Itertools;

use crate::MeshGraph;

/// Single triangle
#[derive(Debug, Clone, Copy)]
pub struct Triangle(pub Vec3, pub Vec3, pub Vec3);

/// Single quad consisting of two triangles
#[derive(Debug, Clone, Copy)]
pub struct Quad(pub Vec3, pub Vec3, pub Vec3, pub Vec3);

/// Subdivided icosahedron to approximate a sphere
#[derive(Debug, Clone, Copy)]
pub struct IcoSphere {
    pub radius: f32,
    pub subdivisions: u8,
}

impl From<Triangle> for MeshGraph {
    fn from(triangle: Triangle) -> Self {
        let Triangle(a, b, c) = triangle;
        MeshGraph::indexed_triangles(&[a, b, c], &[0, 1, 2])
    }
}

impl From<Quad> for MeshGraph {
    fn from(quad: Quad) -> Self {
        let Quad(a, b, c, d) = quad;
        MeshGraph::indexed_triangles(&[a, b, c, d], &[0, 1, 2, 0, 2, 3])
    }
}

impl From<IcoSphere> for MeshGraph {
    fn from(isosphere: IcoSphere) -> Self {
        let IcoSphere {
            radius,
            subdivisions,
        } = isosphere;

        let t = (1.0 + 5_f32.sqrt()) / 2.0;

        // Create initial vertices of an icosahedron
        let mut vertices = vec![
            vec3(-1.0, t, 0.0),
            vec3(1.0, t, 0.0),
            vec3(-1.0, -t, 0.0),
            vec3(1.0, -t, 0.0),
            vec3(0.0, -1.0, t),
            vec3(0.0, 1.0, t),
            vec3(0.0, -1.0, -t),
            vec3(0.0, 1.0, -t),
            vec3(t, 0.0, -1.0),
            vec3(t, 0.0, 1.0),
            vec3(-t, 0.0, -1.0),
            vec3(-t, 0.0, 1.0),
        ];

        // Normalize vertices to make them lie on the sphere surface
        for v in &mut vertices {
            *v = v.normalize();
        }

        // Define the 20 faces of the icosahedron
        let mut faces = vec![
            // 5 faces around point 0
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            // 5 adjacent faces
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            // 5 faces around point 3
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            // 5 adjacent faces
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];

        for _ in 0..subdivisions {
            let mut new_faces = Vec::new();

            // Temporary map to store midpoints of edges
            let mut midpoints = std::collections::HashMap::new();

            for face in &faces {
                // Get the three vertices of the face
                let v1 = face[0];
                let v2 = face[1];
                let v3 = face[2];

                // Get or create the midpoints
                let mid12_idx = {
                    let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
                    if let Some(&idx) = midpoints.get(&edge) {
                        idx
                    } else {
                        let mid_point = (vertices[v1] + vertices[v2]).normalize();
                        let idx = vertices.len();
                        vertices.push(mid_point);
                        midpoints.insert(edge, idx);
                        idx
                    }
                };

                let mid23_idx = {
                    let edge = if v2 < v3 { (v2, v3) } else { (v3, v2) };
                    if let Some(&idx) = midpoints.get(&edge) {
                        idx
                    } else {
                        let mid_point = (vertices[v2] + vertices[v3]).normalize();
                        let idx = vertices.len();
                        vertices.push(mid_point);
                        midpoints.insert(edge, idx);
                        idx
                    }
                };

                let mid31_idx = {
                    let edge = if v3 < v1 { (v3, v1) } else { (v1, v3) };
                    if let Some(&idx) = midpoints.get(&edge) {
                        idx
                    } else {
                        let mid_point = (vertices[v3] + vertices[v1]).normalize();
                        let idx = vertices.len();
                        vertices.push(mid_point);
                        midpoints.insert(edge, idx);
                        idx
                    }
                };

                // Create 4 new triangular faces
                new_faces.push([v1, mid12_idx, mid31_idx]);
                new_faces.push([v2, mid23_idx, mid12_idx]);
                new_faces.push([v3, mid31_idx, mid23_idx]);
                new_faces.push([mid12_idx, mid23_idx, mid31_idx]);
            }

            faces = new_faces;
        }

        MeshGraph::indexed_triangles(
            &vertices.into_iter().map(|v| v * radius).collect_vec(),
            &faces.into_iter().flatten().collect_vec(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_conversion() {
        let triangle = Triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        let mesh_graph = MeshGraph::from(triangle);

        assert_eq!(mesh_graph.vertices.len(), 3);
        assert_eq!(mesh_graph.faces.len(), 1);
        assert_eq!(mesh_graph.halfedges.len(), 3);
    }

    #[test]
    fn test_quad_conversion() {
        let quad = Quad(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        let mesh_graph = MeshGraph::from(quad);

        assert_eq!(mesh_graph.vertices.len(), 4);
        assert_eq!(mesh_graph.faces.len(), 2);
        assert_eq!(mesh_graph.halfedges.len(), 6);
    }
}
