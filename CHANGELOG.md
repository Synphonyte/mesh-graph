# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2025-05-19

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
