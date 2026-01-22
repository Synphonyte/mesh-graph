use glam::Vec3;
use slotmap::SecondaryMap;
use tracing::{error, instrument};

use crate::MeshGraph;

#[cfg(feature = "bevy")]
pub mod bevy;
#[cfg(feature = "gltf")]
pub mod gltf;

/// Classical indexed mesh representation
#[derive(Clone, Debug)]
pub struct VertexIndexBuffers {
    /// Vertex positions, one per vertex.
    pub positions: Vec<Vec3>,
    /// Vertex normals, one per vertex.
    pub normals: Vec<Vec3>,
    /// Indices: 3*N where N is the number of triangles. Indices point to
    /// elements of `positions` and `normals`.
    pub indices: Vec<u32>,
}

impl From<MeshGraph> for VertexIndexBuffers {
    #[instrument(skip(mesh_graph))]
    fn from(mut mesh_graph: MeshGraph) -> VertexIndexBuffers {
        if mesh_graph.vertex_normals.is_none() {
            mesh_graph.compute_vertex_normals();
        }
        let vertex_normals = mesh_graph.vertex_normals.as_ref().unwrap();

        let mut vertex_id_to_index = SecondaryMap::default();
        let mut positions = vec![];
        let mut normals = vec![];
        let mut indices = vec![];

        for (vertex_id, pos) in &mesh_graph.positions {
            vertex_id_to_index.insert(vertex_id, positions.len() as u32);
            positions.push(*pos);
            normals.push(vertex_normals.get(vertex_id).copied().unwrap_or_else(|| {
                error!("Normal not found");
                Vec3::ZERO
            }));
        }

        for face in mesh_graph.faces.values() {
            for vertex in face.vertices(&mesh_graph) {
                let index = vertex_id_to_index[vertex];
                indices.push(index);
            }
        }

        VertexIndexBuffers {
            indices,
            positions,
            normals,
        }
    }
}
