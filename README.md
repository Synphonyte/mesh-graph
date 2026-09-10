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

#### Debugging topology corruption

The `instrumentation` Cargo feature compiles in extra topology probes: chain /
twin / outgoing-list validators that run at the end of every topology op and
report the first op to corrupt the mesh, plus JSON state dump/resume via
`MeshGraph::save_state` / `MeshGraph::load_state`. Enabling the feature enables
the probes — there is no second switch to forget — and regular builds compile
none of it.

The validators scan every halfedge at the end of every topology op, so the cost
grows with the mesh: roughly 30 ms per op call on a 250k-halfedge mesh. That is
nothing for the per-stroke `*_until_*` ops but very noticeable for a host that
calls `merge_vertices_one_rings` hundreds of times per stroke. The further
extras stay opt-in via the environment:

- `MESH_GRAPH_STATE_HISTORY_LEN=<n>` keeps a ring of the last `n` verified mesh
  states (a full mesh clone per op) and writes it to disk when a probe fires, so
  the run can be resumed from any state leading up to the corruption. Default `0`.
- `MESH_GRAPH_HOLE_CHECK=1` adds the boundary-delta probe: every probed op must
  leave the set of open edges exactly as it found it.
- `MESH_GRAPH_TRACE=1` records a ring of the structural re-wiring events leading
  up to a report.

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
