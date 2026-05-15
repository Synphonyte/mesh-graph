use std::path::Path;

use anyhow::{Context, Result, anyhow};
use gltf::{Semantic, accessor::Item};
use itertools::Itertools;
use slotmap::SecondaryMap;

use crate::{MeshGraph, VertexId};

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

pub fn load_with_custom_attribute<T>(
    file: impl AsRef<Path>,
    custom_attr_name: impl AsRef<str>,
) -> Result<(MeshGraph, SecondaryMap<VertexId, T>)>
where
    T: Item + Clone + std::fmt::Debug,
{
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
        .collect_vec();

    let extras: Vec<T> = primitive
        .get(&Semantic::Extras(custom_attr_name.as_ref().to_string()))
        .and_then(|accessor| {
            gltf::accessor::Iter::new(accessor, |buffer| Some(&buffers[buffer.index()]))
        })
        .ok_or(anyhow!(
            "Failed to read custom attribute `{}`",
            custom_attr_name.as_ref()
        ))?
        .collect();

    Ok(MeshGraph::indexed_triangles_with_custom_attribute(
        &pos_iter.map(|p| p.into()).collect_vec(),
        &indices,
        &extras,
    ))
}
