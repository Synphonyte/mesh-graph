# Mesh Graph

[![Crates.io](https://img.shields.io/crates/v/mesh-graph.svg)](https://crates.io/crates/mesh-graph)
[![Docs](https://docs.rs/mesh-graph/badge.svg)](https://docs.rs/mesh-graph/)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/synphonyte/mesh-graph#license)
[![Build Status](https://github.com/synphonyte/mesh-graph/actions/workflows/cd.yml/badge.svg)](https://github.com/synphonyte/mesh-graph/actions/workflows/cd.yml)

<!-- cargo-rdme start -->

MeshGraph is a halfedge data structure for representing triangle meshes.

This is heavily inspired by [SMesh](https://github.com/Bendzae/SMesh) and
[OpenMesh](https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh).

### Features

- Fast spatial queries using parry3d's Bvh
- High performance using slotmap
- Easy integration with Bevy game engine using the `bevy` Cargo feature
- Good debugging using `rerun` Cargo feature to enable the Rerun integration
- Best in class documentation with illustrations

### Usage

```rust
use mesh_graph::{MeshGraph, primitives::IcoSphere};

// Create a new mesh
let mesh_graph = MeshGraph::from(IcoSphere { radius: 10.0, subdivisions: 2 });

// Get some vertex ID and its vertex node
let (vertex_id, vertex) = mesh_graph.vertices.iter().next().unwrap();

// Iterate over all outgoing halfedges of the vertex
for halfedge_id in vertex.outgoing_halfedges(&mesh_graph) {
    // do sth
}

// Get the position of the vertex
let position = mesh_graph.positions[vertex_id];
```

Check out the crate [freestyle-sculpt](https://github.com/Synphonyte/freestyle-sculpt) for
a heavy duty example.

### Connectivity

#### Halfedge

<img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/all.svg" alt="Connectivity" style="max-width: 28em" />

#### Vertex

<img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/all.svg" alt="Connectivity" style="max-width: 50em" />

<!-- cargo-rdme end -->
