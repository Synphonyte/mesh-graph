# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

- Added `zip_coincident_patches`: zips sheets that lie exactly face to face (both layers
  removed, coincident vertices fused). For welds, before merging. Never opens boundary
- Added `collapse_selected_until_edges_above_min_length` and
  `subdivide_selected_until_edges_below_max_length`, which restrict the edge-length
  cleanups to a `Selection` and keep that selection in step with the mesh they reshape:
  elements the operation destroys are dropped from it and elements it creates are added,
  so it never holds a dead id. The scope is `Selection::resolve_to_halfedges`, so a
  selected face puts all three of its edges in scope. The whole-mesh entry points are
  unchanged
- Added `ScopedCleanup`, returned by both scoped operations: the `EdgeLengthCleanup`
  outcome plus `added`/`removed` `SelectionEdit`s. Replaying `removed` then `added` onto
  the selection an operation started from reproduces the one it ended with, so a caller
  mirroring the selection elsewhere can follow along without re-scanning the mesh
- Added `Selection::retain_live`, which drops ids that no longer name a live element and
  returns exactly which ones. Removals are derived from the mesh's own keys rather than
  recorded per operation, so no removal site can be missed
- `SubdivideEdge`'s fields are now public and it derives `Debug`. The struct was public
  with private fields and no accessors, so `subdivide_edge` told external callers nothing
  about what it had created
- `merge_vertices_one_rings` plans its band from the boundary the star removals actually
  open (the rim), measured from the boundary it found at entry, and builds that plan when
  it consumes the whole rim. The old planner, which paired the pre-removal one-rings, is
  gone: where the rim plan applied it was rarely complete, and the cleanup already
  repairs the rest. It closes the extracted weld cases that used to leave
  holes, closes whatever hole its own removals open on every exit path (including the
  flip-and-collapse and failed-pairing exits), and welds the parallel edges a band can
  leave. Its boundary bookkeeping covers only the region within three edges of the two
  vertices, so it adds no cost that grows with the mesh
- `remove_face` finds the face's halfedges from the face itself instead of scanning
  every halfedge in the mesh, so its cost no longer grows with mesh size. Faces with
  broken `next` chains are still removed completely, and the removal order is
  unchanged. Together with the scoped boundary bookkeeping, a
  `merge_vertices_one_rings` call costs the same at every mesh size: about 0.1 ms on a
  983k-halfedge sphere, down from 10 ms
- `merge_vertices_one_rings` no longer leaves dead ids in the caller's marked sets, even
  when a set is kept across many calls, and no longer reports an id as both added and
  removed
- `collapse_until_edges_above_min_length` (and its selection-scoped variant) skips edges
  whose collapse would break the link condition, boundary edges included, and closes
  the holes its own removals open with the same closer as the merge
- Every map and set in the crate uses a fixed-seed hasher, so the same input produces
  the same ids and the same mesh on every run. New dependency: `foldhash`
- Updated `bevy` to `0.20`, `parry3d` to `0.31` and `rerun` to `0.40`. The Bevy 0.20
  bump required no source changes; parry's composite point query now reports the
  sub-shape separately, so `MeshGraph`'s projections carry the triangle that answered
  in `PointProjection::subshape`
- `Mesh::from(MeshGraph)` no longer inserts an empty `ATTRIBUTE_NORMAL` when the graph
  has no vertex normals; Bevy renders such a mesh with its default normal instead of
  receiving a normal attribute shorter than the vertex count

### Breaking Changes

- Every public `HashMap`/`HashSet` is now `mesh_graph::hash::HashMap`/`HashSet`
  (hashbrown with `foldhash::fast::FixedState`) instead of hashbrown's default-hasher
  types: the marked sets of `merge_vertices_one_rings`,
  `collapse_until_edges_above_min_length` and `subdivide_until_edges_below_max_length`,
  `Selection`'s fields and return values, `MeshGraph::index_to_face_id`,
  `halfedges_map` and the attribute maps of the integrations. Build them with
  `mesh_graph::hash::HashSet::default()` (or `HashMap::default()`); `new()` and
  `with_capacity()` do not exist for a non-default hasher

## [0.11.0] - 2026-09-13

- `collapse_until_edges_above_min_length` and `subdivide_until_edges_below_max_length`
  now pick the next edge from a binary heap, resolving equal-length ties
  deterministically and running 1.3–2.4x faster on production scans
- `collapse_until_edges_above_min_length` skips re-testing edges the inversion guard
  rejected unless an endpoint has moved, halving collapse time on the worst production
  mesh without losing any collapses
- `subdivide_until_edges_below_max_length` no longer panics on non-positive lengths, its
  work bound now scales with refinement depth, and it gained the tests it previously
  lacked
- Added a direct dependency on `ordered-float` (already in the tree via `parry3d`, no
  extra build)
- `collapse_until_edges_above_min_length` and `subdivide_until_edges_below_max_length`
  no longer hand back dead ids in their marked sets, so callers only receive live keys
- `Selection::resolve_to_halfedges` and `resolve_to_vertices` no longer panic on ids
  invalidated by an earlier operation; they skip and log instead
- `halfedges_map` no longer abandons the scan at the first twinless halfedge, which
  previously dropped edges from the caller's work set and could let a cleanup loop
  report `Converged` while the mesh still violated the threshold

### Breaking Changes

- `collapse_until_edges_above_min_length` and `subdivide_until_edges_below_max_length`
  now return `EdgeLengthCleanup` instead of `()`, reporting whether every edge ended up
  inside the length band

## [0.10.0] - 2026-09-10

- Hardened against panics and infinite loops.
- Fixed topology corruption in `collapse_until_edges_above_min_length`,
  `subdivide_until_edges_below_max_length` and `merge_vertices_one_rings`: dangling
  halfedge twins, faces with broken chains and holes left behind by one-ring merges.
- Added the optional `instrumentation` Cargo feature: topology validators, an op
  trace, a state-history ring and JSON state dump/resume (`MeshGraph::save_state` /
  `MeshGraph::load_state`) for debugging mesh corruption. Off by default; enabling
  the feature enables the validators. The costlier extras are opt-in at runtime with
  `MESH_GRAPH_STATE_HISTORY_LEN`, `MESH_GRAPH_HOLE_CHECK` and `MESH_GRAPH_TRACE`.

### Breaking Changes

- `MeshGraph::triangles` now returns `Option<Self>` instead of `Self`. It returns `None` if the number of vertex positions is not a multiple of 3 instead of panicking.
- `Face::normal_from_positions` now returns `Option<Vec3>` instead of `Vec3`. It returns `None` if there are fewer than 3 positions or the normal cannot be normalized.
- `Polygon2::terminal` now returns `Option<Vec2>` instead of `Vec2`. It returns `None` if the polygon is empty.

## [0.9.0] - 2026-08-31

- Updated dependencies `manifold-csg` to `0.4`, `parry3d` to `0.30` and `rerun` to `0.36`

## [0.8.0] - 2026-07-27

- Update dependencies `bevy` to `0.19` and `parry3d` to `0.29`

## [0.7.1] - 2026-07-27

- Added integration with `manifold-csg`
- Prevented potential endless loops

## [0.7.0] - 2026-06-17

- Updated dependencies rerun, parry3d, itertools and glam
- Fixed `merge_one_ring` edge cases
- A bunch of `edge` methods now return `Option<...>` because they can fail if given invalid `VertexId`s.
- Added `MeshGraph::merge_vertices`

## [0.6.0] - 2026-04-10

- Updated dependencies rerun and hashbrown

## [0.5.0] - 2026-03-14

- Updated parry and bevy dependencies
- All methods to add stuff to the mesh graph are now named `add_...`
- Similarly, all methods to remove stuff from the mesh graph are now named `remove_...`
- Added a bunch of methods to access and modify for example `merge_vertices_one_rings`

## [0.4.0] - 2025-12-22

- Changed the collapse edges to get min length edges to check if a potential edge collapse would
  lead to a self-intersection and then doesn't collapse it.
- Added `Face::normal()` and `Face::is_degenerate()`.
- Made `MeshGraph` construction from indexes vertices more robust.
  - To that end there is now a `MeshGraph::insert_or_get_edge()`.
  - And `MeshGraph::insert_face()` now requires all three halfedges as arguments.
- Implemented `MeshGraph::make_outgoing_halfedge_boundary_if_possible()` and added `MeshGraph::make_all_outgoing_halfedges_boundary_if_possible()`.

## [0.3.2] - 2025-10-16

- Fixed bevy integration (thanks to @madmaxio).
- Made `compute_transform_from_plane_into_xy` public.
- Added `Selection::grow()`.

## [0.3.1] - 2025-10-08

- Fixed BVH (re)building

## [0.3.0] (yanked) - 2025-10-07

- Made the mesh graph API a lot less prone to panic at the cost of some `Option` return types.
- Added `CircularHalfedgesIterator`.
- Updated dependencies

## [0.2.2] - 2025-07-22

- Added plane slicing

## [0.2.1] - 2025-06-23

- Added cleanup of zero faces when constructing a mesh graph from vertices

## [0.2.0] - 2025-05-29

- Added mesh graph creation from a triangle list
- Updated dependency parry3d to version 0.21
- Added serde support behind feature flag `serde`
- Renamed dissolve to collapse as it better describes the operation
- Fixed the cleanup algorithm after edge collapse

## [0.1.1] - 2025-05-14

- Walked back glam to version 0.29 to stay compatible with stuff

## [0.1.0] (yanked) - 2025-05-14

- Halfedge graph implementation
- Optional vertex normals
- Split edge
- Collapse edge
- Selection
- Bevy integration
- Rerun debug
