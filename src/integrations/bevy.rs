use bevy::{
    asset::RenderAssetUsages,
    mesh::{Indices, PrimitiveTopology},
    prelude::*,
};

use crate::MeshGraph;

use super::VertexIndexBuffers;

impl From<MeshGraph> for Mesh {
    fn from(mesh_graph: MeshGraph) -> Self {
        let buffers: VertexIndexBuffers = mesh_graph.into();
        let vertex_count = buffers.positions.len();

        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            buffers
                .positions
                .into_iter()
                .map(|v| v.to_array())
                .collect::<Vec<_>>(),
        )
        .with_inserted_indices(Indices::U32(buffers.indices));

        // `normals` is empty unless the graph has vertex normals. Inserting it anyway
        // would give the mesh a normal attribute shorter than its vertex count, which
        // Bevy rejects when it builds the vertex layout. Without the attribute Bevy
        // falls back to its default normal.
        if !buffers.normals.is_empty() && buffers.normals.len() == vertex_count {
            mesh.insert_attribute(
                Mesh::ATTRIBUTE_NORMAL,
                buffers
                    .normals
                    .into_iter()
                    .map(|v| v.to_array())
                    .collect::<Vec<_>>(),
            );
        }

        // TODO ?
        // if buffers.uvs.len() == vertex_count {
        //     mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, buffers.uvs);
        // }

        mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn converts_to_a_bevy_mesh() {
        let mut mesh_graph = MeshGraph::new();
        mesh_graph.add_face_from_positions(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        mesh_graph.compute_vertex_normals();

        let mesh = Mesh::from(mesh_graph);

        assert_eq!(mesh.count_vertices(), 3);
        assert_eq!(mesh.indices().unwrap().len(), 3);
        assert_eq!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap().len(), 3);
        assert_eq!(mesh.attribute(Mesh::ATTRIBUTE_NORMAL).unwrap().len(), 3);
    }

    #[test]
    fn omits_the_normal_attribute_when_the_graph_has_none() {
        let mut mesh_graph = MeshGraph::new();
        mesh_graph.add_face_from_positions(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        let mesh = Mesh::from(mesh_graph);

        assert_eq!(mesh.count_vertices(), 3);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_NORMAL).is_none());
    }
}
