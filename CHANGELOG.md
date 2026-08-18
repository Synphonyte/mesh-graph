# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 

- Hardened against panics and infinite loops.

### Breaking Changes

- `MeshGraph::triangles` now returns `Option<Self>` instead of `Self`. It returns `None` if the number of vertex positions is not a multiple of 3 instead of panicking.
- `Face::normal_from_positions` now returns `Option<Vec3>` instead of `Vec3`. It returns `None` if there are fewer than 3 positions or the normal cannot be normalized.
- `Polygon2::terminal` now returns `Option<Vec2>` instead of `Vec2`. It returns `None` if the polygon is empty.
- `MeshGraph::collapse_until_edges_above_min_length` now takes an additional `max_iterations` parameter to prevent potential infinite loops.
- `MeshGraph::subdivide_until_edges_below_max_length` now takes an additional `max_iterations` parameter to prevent potential infinite loops.
- Updated dependencies `manifold-csg` to `0.4`, `parry3d` to `0.30` and `rerun` to `0.36`.

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
