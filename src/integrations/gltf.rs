use std::path::Path;

use anyhow::{Context, Result, anyhow};
use itertools::Itertools;

use crate::MeshGraph;

pub fn load(file: impl AsRef<Path>) -> Result<MeshGraph> {
    let (document, buffers, _) = gltf::import(&file).context("Failed to load GLTF file")?;

    let mesh = document
        .meshes()
        .next()
        .ok_or(anyhow!("Document has no meshes"))?;
    let primitive = mesh
        .primitives()
        .next()
        .ok_or(anyhow!("Mesh has no primitives"))?;
    let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
    let pos_iter = reader
        .read_positions()
        .ok_or(anyhow!("Failed to read positions"))?;
    let indices = reader
        .read_indices()
        .ok_or(anyhow!("Failed to read indices"))?
        .into_u32()
        .map(|i| i as usize)
        .collect::<Vec<_>>();

    Ok(MeshGraph::indexed_triangles(
        &pos_iter.map(|p| p.into()).collect_vec(),
        &indices,
    ))
}
