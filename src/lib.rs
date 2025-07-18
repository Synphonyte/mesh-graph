//! MeshGraph is a halfedge data structure for representing triangle meshes.
//! It uses parry3d's Qbvh to implement some of parry3d's spatial queries.
//! It also uses slotmap to manage the graph nodes.
//!
//! This is heavily inspired by [SMesh](https://github.com/Bendzae/SMesh) and
//! [OpenMesh](https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh).
//!
//! ## Features
//!
//! - Fast spatial queries using parry3d's Qbvh
//! - High performance using slotmap
//! - Easy integration with Bevy game engine using the `bevy` Cargo feature
//! - Good debugging using `rerun` Cargo feature to enable the Rerun integration
//! - Nice documentation with illustrations
//!
//! ## Usage
//!
//! ```
//! use mesh_graph::{MeshGraph, primitives::IcoSphere};
//!
//! // Create a new mesh
//! let mesh_graph = MeshGraph::from(IcoSphere { radius: 10.0, subdivisions: 2 });
//!
//! // Get some vertex ID and its vertex node
//! let (vertex_id, vertex) = mesh_graph.vertices.iter().next().unwrap();
//!
//! // Iterate over all outgoing halfedges of the vertex
//! for halfedge_id in vertex.outgoing_halfedges(&mesh_graph) {
//!     // do sth
//! }
//!
//! // Get the position of the vertex
//! let position = mesh_graph.positions[vertex_id];
//! ```
//!
//! Check out the crate [freestyle-sculpt](https://github.com/Synphonyte/freestyle-sculpt) for
//! a heavy duty example.
//!
//! ## Connectivity
//!
//! ### Halfedge
//!
//! <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/halfedge/all.svg" alt="Connectivity" style="max-width: 28em" />
//!
//! ### Vertex
//!
//! <img src="https://raw.githubusercontent.com/Synphonyte/mesh-graph/refs/heads/main/docs/vertex/all.svg" alt="Connectivity" style="max-width: 50em" />

mod elements;
pub mod integrations;
mod ops;
pub mod primitives;
mod selection;
#[cfg(feature = "rerun")]
pub mod utils;

pub use elements::*;
pub use selection::*;

use hashbrown::HashMap;
use itertools::Itertools;
use parry3d::{
    math::Point,
    partitioning::{Qbvh, QbvhUpdateWorkspace},
    shape::Triangle,
};

use glam::Vec3;
use slotmap::{SecondaryMap, SlotMap};

#[cfg(feature = "rerun")]
lazy_static::lazy_static! {
    pub static ref RR: rerun::RecordingStream = rerun::RecordingStreamBuilder::new("mesh_graph").spawn().unwrap();
}

/// Halfedge data structure for representing triangle meshes.
///
/// Please see the [crate documentation](crate) for more information.
#[derive(Clone)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MeshGraph {
    /// Acceleration structure for fast spatial queries. Uses parry3d's Qbvh to implement some of parry3d's spatial queries.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub qbvh: Qbvh<Face>,
    /// Used in conjunction with the QBVH to accelerate spatial queries.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub qbvh_workspace: QbvhUpdateWorkspace,
    /// Used to map indices stored in the QBVH to face IDs.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub index_to_face_id: Vec<FaceId>,

    /// Maps vertex IDs to their corresponding graph node
    pub vertices: SlotMap<VertexId, Vertex>,
    /// Maps halfedge IDs to their corresponding graph node
    pub halfedges: SlotMap<HalfedgeId, Halfedge>,
    /// Maps face IDs to their corresponding graph node
    pub faces: SlotMap<FaceId, Face>,

    /// Maps vertex IDs to their corresponding positions
    pub positions: SecondaryMap<VertexId, Vec3>,
    /// Maps vertex IDs to their corresponding normals
    pub vertex_normals: Option<SecondaryMap<VertexId, Vec3>>,
}

impl MeshGraph {
    /// Create a triangle mesh graph from vertex positions.
    /// Every three positions represent a triangle.
    ///
    /// Vertices with the same position are merged into a single vertex.
    pub fn triangles(vertex_positions: &[Vec3]) -> Self {
        assert!(
            vertex_positions.len() % 3 == 0,
            "Number of vertex positions should be a multiple of 3"
        );

        // Create a map to track unique vertices
        let mut unique_positions: Vec<Vec3> = Vec::with_capacity(vertex_positions.len() / 3);
        let mut face_indices = Vec::with_capacity(vertex_positions.len());

        for vertex_pos in vertex_positions {
            // Check if we've seen this position before using a fuzzy float comparison
            let mut idx = None;
            for (j, pos) in unique_positions.iter().enumerate() {
                const EPSILON: f32 = 1e-5;

                if pos.distance_squared(*vertex_pos) < EPSILON {
                    idx = Some(j);
                    break;
                }
            }

            // Use the existing index or add a new vertex
            let vertex_idx = if let Some(idx) = idx {
                idx
            } else {
                let new_idx = unique_positions.len();
                unique_positions.push(*vertex_pos);

                #[cfg(feature = "rerun")]
                RR.log(
                    "meshgraph/construct/vertices",
                    &rerun::Points3D::new(unique_positions.iter().map(crate::utils::vec3_array)),
                )
                .unwrap();

                new_idx
            };

            // Add to face indices
            face_indices.push(vertex_idx);
        }

        // Use indexed_triangles to create the mesh
        Self::indexed_triangles(&unique_positions, &face_indices)
    }

    /// Create a triangle mesh graph from vertex positions and face indices.
    /// Every chunk of three indices represents a triangle.
    pub fn indexed_triangles(vertex_positions: &[Vec3], face_indices: &[usize]) -> Self {
        let mut graph = Self {
            qbvh: Qbvh::new(),
            qbvh_workspace: QbvhUpdateWorkspace::default(),
            index_to_face_id: Vec::with_capacity(face_indices.len() / 3),

            vertices: SlotMap::with_capacity_and_key(vertex_positions.len()),
            halfedges: SlotMap::with_capacity_and_key(face_indices.len()),
            faces: SlotMap::with_capacity_and_key(face_indices.len() / 3),

            positions: SecondaryMap::with_capacity(vertex_positions.len()),
            vertex_normals: None,
        };

        let mut vertex_ids = Vec::with_capacity(vertex_positions.len());

        for pos in vertex_positions {
            vertex_ids.push(graph.insert_vertex(*pos));
        }

        let mut start_end_vertex_to_halfedge =
            HashMap::<(VertexId, VertexId), HalfedgeId>::default();

        for chunk in face_indices.chunks_exact(3) {
            let a = vertex_ids[chunk[0]];
            let b = vertex_ids[chunk[1]];
            let c = vertex_ids[chunk[2]];

            if a == b || b == c || c == a {
                #[cfg(feature = "rerun")]
                RR.log(
                    "meshgraph/construct/zero_face",
                    &rerun::Points3D::new(
                        [graph.positions[a], graph.positions[b], graph.positions[c]]
                            .iter()
                            .map(crate::utils::vec3_array),
                    ),
                )
                .unwrap();

                continue;
            }

            let he_a_id = graph.insert_halfedge(b);
            let he_b_id = graph.insert_halfedge(c);
            let he_c_id = graph.insert_halfedge(a);

            start_end_vertex_to_halfedge.insert((b, c), he_b_id);
            start_end_vertex_to_halfedge.insert((c, a), he_c_id);

            let face_id = graph.insert_face(he_a_id);

            let he_a = &mut graph.halfedges[he_a_id];
            he_a.next = Some(he_b_id);
            he_a.face = Some(face_id);

            if let Some(twin_a_id) = start_end_vertex_to_halfedge.get(&(b, a)) {
                he_a.twin = Some(*twin_a_id);
                graph.halfedges[*twin_a_id].twin = Some(he_a_id);
            } else {
                start_end_vertex_to_halfedge.insert((a, b), he_a_id);
            }

            let he_b = &mut graph.halfedges[he_b_id];
            he_b.next = Some(he_c_id);
            he_b.face = Some(face_id);

            if let Some(twin_b_id) = start_end_vertex_to_halfedge.get(&(c, b)) {
                he_b.twin = Some(*twin_b_id);
                graph.halfedges[*twin_b_id].twin = Some(he_b_id);
            } else {
                start_end_vertex_to_halfedge.insert((b, c), he_b_id);
            }

            let he_c = &mut graph.halfedges[he_c_id];
            he_c.next = Some(he_a_id);
            he_c.face = Some(face_id);

            if let Some(twin_c_id) = start_end_vertex_to_halfedge.get(&(a, c)) {
                he_c.twin = Some(*twin_c_id);
                graph.halfedges[*twin_c_id].twin = Some(he_c_id);
            } else {
                start_end_vertex_to_halfedge.insert((c, a), he_c_id);
            }

            graph.vertices[a].outgoing_halfedge = Some(he_a_id);
            graph.vertices[b].outgoing_halfedge = Some(he_b_id);
            graph.vertices[c].outgoing_halfedge = Some(he_c_id);
        }

        graph.rebuild_qbvh();

        graph
    }

    /// Computes the vertex normals by averaging over the computed face normals
    pub fn compute_vertex_normals(&mut self) {
        let mut normals = SecondaryMap::with_capacity(self.vertices.len());

        for face in self.faces.values() {
            let ha_a_id = face.halfedge;
            let he_a = self.halfedges[ha_a_id];

            let he_b_id = he_a
                .next
                .expect("Halfedge has definitely a face and thus a next halfedge");
            let he_b = self.halfedges[he_b_id];

            let a = he_a.start_vertex(self);
            let b = he_a.end_vertex;
            let c = he_b.end_vertex;

            let diff_a = self.positions[c] - self.positions[a];
            let diff_b = self.positions[c] - self.positions[b];

            // TODO : normalizing necessary here?
            let face_normal = diff_a.cross(diff_b);

            *normals.entry(a).unwrap().or_default() += face_normal;
            *normals.entry(b).unwrap().or_default() += face_normal;
            *normals.entry(c).unwrap().or_default() += face_normal;
        }

        self.vertex_normals = Some(normals);
        self.normalize_vertex_normals();
    }

    /// Ensures that the vertex normals are all normalized
    pub fn normalize_vertex_normals(&mut self) {
        if let Some(normals) = &mut self.vertex_normals {
            for normal in normals.values_mut() {
                *normal = normal.normalize();
            }
        }
    }

    pub fn rebalance_qbvh(&mut self) {
        self.qbvh.rebalance(0.0, &mut self.qbvh_workspace);
    }

    /// Recomputes the bounding boxes of the QBVH. This is necessary when the mesh is modified.
    ///
    /// It does not no change the topology of the QBVH. Call `rebalance_qbvh` to to that.
    pub fn refit_qbvh(&mut self) {
        let mesh_graph = self.clone();

        self.qbvh.refit(0.0, &mut self.qbvh_workspace, |face| {
            let pos = face
                .vertices(&mesh_graph)
                .into_iter()
                .map(|v| {
                    let Vec3 { x, y, z } = mesh_graph.positions[v];
                    Point::new(x, y, z)
                })
                .collect_vec();

            Triangle::new(pos[0], pos[1], pos[2]).local_aabb()
        });
    }

    fn recount_faces(&mut self) {
        self.index_to_face_id.clear();
        for (id, face) in &mut self.faces {
            face.index = self.index_to_face_id.len();
            self.index_to_face_id.push(id);
        }
    }

    /// Clear and rebuild the QBVH.
    pub fn rebuild_qbvh(&mut self) {
        self.recount_faces();

        let data = self
            .faces
            .values()
            .map(|face| {
                let pos = face
                    .vertices(self)
                    .into_iter()
                    .map(|v| {
                        let Vec3 { x, y, z } = self.positions[v];
                        Point::new(x, y, z)
                    })
                    .collect_vec();

                (*face, Triangle::new(pos[0], pos[1], pos[2]).local_aabb())
            })
            .collect_vec();

        self.qbvh.clear_and_rebuild(data.into_iter(), 0.0);
    }
}

#[cfg(feature = "rerun")]
impl MeshGraph {
    pub fn log_selection_rerun(&self, name: &str, selection: &Selection) {
        use crate::RR;
        use crate::utils::*;

        RR.log(
            format!("meshgraph/selection/{name}/points"),
            &rerun::Points3D::new(
                selection
                    .vertices
                    .iter()
                    .map(|v| vec3_array(self.positions[*v]))
                    .collect_vec(),
            ),
        )
        .unwrap();

        self.log_hes_rerun_with_name(
            format!("meshgraph/selection/{name}/halfedges"),
            &selection.halfedges.iter().copied().collect::<Vec<_>>(),
        );

        let face_centers = selection
            .faces
            .iter()
            .map(|face_id| {
                let face = self.faces[*face_id];

                let center = face
                    .vertices(self)
                    .into_iter()
                    .map(|v_id| self.positions[v_id])
                    .reduce(|acc, p| acc + p)
                    .unwrap()
                    / 3.0;

                vec3_array(center)
            })
            .collect_vec();

        RR.log(
            format!("meshgraph/selection/{name}/faces"),
            &rerun::Points3D::new(face_centers),
        )
        .unwrap();
    }

    pub fn log_vert_rerun(&self, name: &str, vert: VertexId) {
        use crate::RR;
        use crate::utils::*;

        let pos = self.positions[vert];

        RR.log(
            format!("meshgraph/vertex/{name}"),
            &rerun::Points3D::new(vec![vec3_array(pos)]),
        )
        .unwrap();
    }

    pub fn log_he_rerun(&self, name: &str, halfedge: HalfedgeId) {
        use crate::RR;
        use crate::utils::*;

        let he = self.halfedges[halfedge];

        let start = self.positions[he.start_vertex(self)];
        let end = self.positions[he.end_vertex];

        RR.log(
            format!("meshgraph/halfedge/{name}"),
            &rerun::Arrows3D::from_vectors([vec3_array(end - start)])
                .with_origins([vec3_array(start)]),
        )
        .unwrap();
    }

    pub fn log_hes_rerun(&self, name: &str, halfedges: &[HalfedgeId]) {
        self.log_hes_rerun_with_name(format!("meshgraph/halfedge/{name}"), halfedges);
    }

    fn log_hes_rerun_with_name(&self, name: String, halfedges: &[HalfedgeId]) {
        use crate::RR;
        use crate::utils::*;

        let mut origins = Vec::with_capacity(halfedges.len());
        let mut vectors = Vec::with_capacity(halfedges.len());

        for &he_id in halfedges {
            let he = self.halfedges[he_id];

            let start = self.positions[he.start_vertex(self)];
            let end = self.positions[he.end_vertex];

            origins.push(vec3_array(start));
            vectors.push(vec3_array(end - start));
        }

        RR.log(
            name,
            &rerun::Arrows3D::from_vectors(vectors).with_origins(origins),
        )
        .unwrap();
    }

    pub fn log_face_rerun(&self, name: &str, face: FaceId) {
        use crate::RR;
        use crate::utils::*;

        let mut origins = Vec::with_capacity(3);
        let mut vectors = Vec::with_capacity(3);

        let face = self.faces[face];

        let pos = face
            .vertices(self)
            .iter()
            .map(|v| self.positions[*v])
            .collect::<Vec<_>>();

        let center = pos.iter().copied().reduce(|a, b| a + b).unwrap() / pos.len() as f32;

        let pos = face
            .vertices(self)
            .into_iter()
            .zip(pos)
            .map(|(v, p)| (v, center * 0.1 + p * 0.9))
            .collect::<HashMap<_, _>>();

        for he_id in face.halfedges(self) {
            let he = self.halfedges[he_id];

            let start = pos[&he.start_vertex(self)];
            let end = pos[&he.end_vertex];

            origins.push(vec3_array(start));
            vectors.push(vec3_array(end - start));
        }

        #[cfg(feature = "rerun")]
        {
            RR.log(
                format!("meshgraph/face/{name}"),
                &rerun::Arrows3D::from_vectors(vectors).with_origins(origins),
            )
            .unwrap();
        }
    }

    pub fn log_rerun(&self) {
        use crate::RR;
        use crate::utils::*;

        let buffers = crate::integrations::VertexIndexBuffers::from(self.clone());
        RR.log(
            "meshgraph/mesh",
            &rerun::Mesh3D::new(
                buffers
                    .positions
                    .into_iter()
                    .zip(buffers.normals.iter().cloned())
                    .map(|(pos, norm)| vec3_array(pos - norm * 0.1)),
            )
            .with_triangle_indices(
                buffers
                    .indices
                    .chunks(3)
                    .map(|chunk| rerun::datatypes::UVec3D::new(chunk[0], chunk[1], chunk[2])),
            )
            .with_vertex_colors(
                buffers
                    .normals
                    .into_iter()
                    .map(|v| {
                        [
                            (100.0 + v.x * 100.0) as u8,
                            (100.0 + v.y * 100.0) as u8,
                            (100.0 + v.z * 100.0) as u8,
                        ]
                    })
                    .collect::<Vec<_>>(),
            ),
        )
        .unwrap();

        RR.log(
            "meshgraph/positions",
            &rerun::Points3D::new(self.positions.values().map(vec3_array))
                .with_radii(self.positions.iter().map(|_| 0.01)),
        )
        .unwrap();

        let mut origins = Vec::with_capacity(self.faces.len() * 3);
        let mut vectors = Vec::with_capacity(self.faces.len() * 3);

        let mut he_to_pos = HashMap::<HalfedgeId, (Vec3, Vec3)>::default();

        for face in self.faces.values() {
            let pos = face
                .vertices(self)
                .iter()
                .map(|v| self.positions[*v])
                .collect::<Vec<_>>();

            let center = pos.iter().copied().reduce(|a, b| a + b).unwrap() / pos.len() as f32;

            let pos = face
                .vertices(self)
                .into_iter()
                .zip(pos)
                .map(|(v, p)| (v, center * 0.1 + p * 0.9))
                .collect::<HashMap<_, _>>();

            for he_id in face.halfedges(self) {
                let he = self.halfedges[he_id];

                let start = pos[&he.start_vertex(self)];
                let end = pos[&he.end_vertex];

                he_to_pos.insert(he_id, (start, end));

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        for (he_id, he) in &self.halfedges {
            if he.is_boundary() {
                let start_vertex = he.start_vertex(self);
                let end_vertex = he.end_vertex;

                let start = self.positions[start_vertex];
                let end = self.positions[end_vertex];

                let offset = if let Some(normals) = self.vertex_normals.as_ref() {
                    normals[start_vertex]
                        .lerp(normals[end_vertex], 0.5)
                        .cross(end - start)
                        .normalize()
                        * 0.1
                } else {
                    Vec3::ZERO
                };

                let (start, end) = (start.lerp(end, 0.1) + offset, end.lerp(start, 0.1) + offset);

                he_to_pos.insert(he_id, (start, end));

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            "meshgraph/halfedges",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for (he_id, he) in &self.halfedges {
            let twin = he.twin();

            let (he_start, he_end) = he_to_pos[&he_id];
            let (tw_start, tw_end) = he_to_pos[&twin];

            let start = he_start * 0.8 + he_end * 0.2;
            let end = tw_start * 0.2 + tw_end * 0.8;

            origins.push(vec3_array(start));
            vectors.push(vec3_array(end - start));
        }

        RR.log(
            "meshgraph/twins",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for (v_id, v) in &self.vertices {
            if let Some(he) = v.outgoing_halfedge.as_ref() {
                let start = self.positions[v_id];

                RR.log(
                    "meshgraph/the_vertex",
                    &rerun::Points3D::new([vec3_array(start)]),
                )
                .unwrap();

                let (start_he, end_he) = he_to_pos[he];

                let end = start_he.lerp(end_he, 0.05);

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            "meshgraph/outgoing_halfedges",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for face in self.faces.values() {
            let start = face.center(self);

            let (he_start, he_end) = he_to_pos[&face.halfedge];
            let end = he_start * 0.6 + he_end * 0.4;

            origins.push(vec3_array(start));
            vectors.push(vec3_array(end - start));
        }

        RR.log(
            "meshgraph/face_halfedges",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for (he_id, he) in &self.halfedges {
            if let Some(face_id) = he.face {
                let (he_start, he_end) = he_to_pos[&he_id];
                let start = he_start * 0.4 + he_end * 0.6;

                let end = self.faces[face_id].center(self);

                origins.push(vec3_array(start));
                vectors.push(vec3_array((end - start) * 0.9));
            }
        }

        RR.log(
            "meshgraph/halfedge_faces",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for (he_id, he) in &self.halfedges {
            if let Some(next_id) = he.next {
                let (he_start, he_end) = he_to_pos[&he_id];
                let start = he_start * 0.15 + he_end * 0.85;

                let (next_start, next_end) = he_to_pos[&next_id];
                let end = next_start * 0.85 + next_end * 0.15;

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            "meshgraph/halfedge_next",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();
    }
}
