use std::iter::repeat;

use glam::Vec3;
use hashbrown::HashMap;
use slotmap::SecondaryMap;
use tracing::{error, instrument};

use crate::{MeshGraph, VertexId};

#[cfg(feature = "bevy")]
pub mod bevy;
#[cfg(feature = "gltf")]
pub mod gltf;

/// Classical indexed mesh representation
#[derive(Clone, Debug)]
pub struct VertexIndexBuffers<T = ()> {
    /// Vertex positions, one per vertex.
    pub positions: Vec<Vec3>,
    /// Vertex normals, one per vertex.
    pub normals: Vec<Vec3>,
    /// Indices: 3*N where N is the number of triangles. Indices point to
    /// elements of `positions` and `normals`.
    pub indices: Vec<u32>,
    /// Potential custom vertex attribute(s)
    pub custom_vertex_attribute: Vec<T>,
}

impl<T> VertexIndexBuffers<T>
where
    T: Clone + Default,
{
    pub fn with_attr_from_map(mesh_graph: &MeshGraph, attr: &HashMap<VertexId, T>) -> Self {
        let (positions, normals, indices, vertex_id_to_index) =
            Self::attrs_from_mesh_graph(mesh_graph);

        let mut custom_vertex_attribute = repeat(T::default())
            .take(positions.len())
            .collect::<Vec<T>>();

        for (vertex_id, value) in attr {
            if let Some(index) = vertex_id_to_index.get(*vertex_id) {
                custom_vertex_attribute[*index as usize] = value.clone();
            }
        }

        Self {
            positions,
            normals,
            indices,
            custom_vertex_attribute,
        }
    }

    fn attrs_from_mesh_graph(
        mesh_graph: &MeshGraph,
    ) -> (Vec<Vec3>, Vec<Vec3>, Vec<u32>, SecondaryMap<VertexId, u32>) {
        let mut vertex_id_to_index = SecondaryMap::default();

        let mut positions = vec![];
        let mut normals = vec![];
        let mut indices = vec![];

        for (vertex_id, pos) in &mesh_graph.positions {
            vertex_id_to_index.insert(vertex_id, positions.len() as u32);
            positions.push(*pos);

            if let Some(vertex_normals) = mesh_graph.vertex_normals.as_ref() {
                normals.push(vertex_normals.get(vertex_id).copied().unwrap_or_else(|| {
                    error!("Normal not found");
                    Vec3::ZERO
                }));
            }
        }

        'outer: for face in mesh_graph.faces.values() {
            let mut face_indices = Vec::with_capacity(3);

            for vertex in face.vertices(&mesh_graph) {
                let Some(&index) = vertex_id_to_index.get(vertex) else {
                    error!("Vertex {vertex:?} not found in mapped vertices");
                    continue 'outer;
                };
                face_indices.push(index);
            }

            indices.extend(face_indices);
        }

        (positions, normals, indices, vertex_id_to_index)
    }
}

impl From<&MeshGraph> for VertexIndexBuffers {
    #[instrument(skip(mesh_graph))]
    fn from(mesh_graph: &MeshGraph) -> VertexIndexBuffers {
        let (positions, normals, indices, _) = Self::attrs_from_mesh_graph(mesh_graph);

        VertexIndexBuffers {
            indices,
            positions,
            normals,
            custom_vertex_attribute: vec![],
        }
    }
}
