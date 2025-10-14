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

        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, buffers.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, buffers.normals)
        .with_inserted_indices(Indices::U32(buffers.indices))

        // TODO ?
        // let vertex_count = buffers.positions.len();
        // if buffers.uvs.len() == vertex_count {
        //     mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, buffers.uvs);
        // }
    }
}
