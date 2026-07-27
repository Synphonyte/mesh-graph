use glam::Vec3;
use itertools::Itertools;

use crate::{MeshGraph, integrations::VertexIndexBuffers};

impl TryFrom<&MeshGraph> for manifold_csg::Manifold {
    type Error = manifold_csg::CsgError;

    fn try_from(value: &MeshGraph) -> Result<Self, Self::Error> {
        let buffers = VertexIndexBuffers::from(value);

        let vert_props = buffers
            .positions
            .iter()
            .flat_map(|p| p.to_array())
            .collect_vec();

        manifold_csg::Manifold::from_mesh_f32(&vert_props, 3, &buffers.indices)
    }
}

impl TryFrom<MeshGraph> for manifold_csg::Manifold {
    type Error = manifold_csg::CsgError;

    fn try_from(value: MeshGraph) -> Result<Self, Self::Error> {
        Self::try_from(&value)
    }
}

impl From<&manifold_csg::Manifold> for MeshGraph {
    fn from(value: &manifold_csg::Manifold) -> Self {
        let (vert_props, n_props, tri_indices) = value.to_mesh_f32();
        println!("n_props: {}", n_props);

        Self::indexed_triangles(
            &vert_props
                .chunks(n_props)
                .map(|vert| Vec3::new(vert[0], vert[1], vert[2]))
                .collect_vec(),
            &tri_indices.iter().map(|i| *i as usize).collect_vec(),
        )
    }
}

impl From<manifold_csg::Manifold> for MeshGraph {
    fn from(value: manifold_csg::Manifold) -> Self {
        Self::from(&value)
    }
}
