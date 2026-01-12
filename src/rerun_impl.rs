use glam::Vec3;
use hashbrown::HashMap;
use itertools::Itertools;

use crate::RR;
use crate::utils::*;
use crate::{FaceId, HalfedgeId, MeshGraph, Selection, VertexId};

impl MeshGraph {
    pub fn log_selection_rerun(&self, name: &str, selection: &Selection) {
        use itertools::Itertools;

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

        let start = self.positions[he.start_vertex(self).unwrap()];
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

    pub fn log_hes_rerun_with_name(&self, name: String, halfedges: &[HalfedgeId]) {
        let mut origins = Vec::with_capacity(halfedges.len());
        let mut vectors = Vec::with_capacity(halfedges.len());

        for &he_id in halfedges {
            let he = self.halfedges[he_id];

            let start = self.positions[he.start_vertex(self).unwrap()];
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

    pub fn log_faces_rerun(&self, name: &str, faces: &[FaceId]) {
        self.log_faces_rerun_with_name(format!("meshgraph/face/{name}"), faces);
    }

    pub fn log_faces_rerun_with_name(&self, name: String, faces: &[FaceId]) {
        let mut origins = Vec::with_capacity(faces.len() * 3);
        let mut vectors = Vec::with_capacity(faces.len() * 3);

        for &face_id in faces {
            let face = self.faces[face_id];

            let pos = face.vertices(self).map(|v| self.positions[v]).collect_vec();

            let center = pos.iter().copied().reduce(|a, b| a + b).unwrap() / pos.len() as f32;

            let pos = face
                .vertices(self)
                .zip(pos)
                .map(|(v, p)| (v, center * 0.1 + p * 0.9))
                .collect::<HashMap<_, _>>();

            for he_id in face.halfedges(self) {
                let he = self.halfedges[he_id];

                let start = pos[&he.start_vertex(self).unwrap()];
                let end = pos[&he.end_vertex];

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            name,
            &rerun::Arrows3D::from_vectors(vectors).with_origins(origins),
        )
        .unwrap();
    }

    pub fn log_face_rerun(&self, name: &str, face: FaceId) {
        let mut origins = Vec::with_capacity(3);
        let mut vectors = Vec::with_capacity(3);

        let face = self.faces[face];

        let pos = face.vertices(self).map(|v| self.positions[v]).collect_vec();

        let center = pos.iter().copied().reduce(|a, b| a + b).unwrap() / pos.len() as f32;

        let pos = face
            .vertices(self)
            .zip(pos)
            .map(|(v, p)| (v, center * 0.1 + p * 0.9))
            .collect::<HashMap<_, _>>();

        for he_id in face.halfedges(self) {
            let he = self.halfedges[he_id];

            let start = pos[&he.start_vertex(self).unwrap()];
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
        // let buffers = crate::integrations::VertexIndexBuffers::from(self.clone());
        // RR.log(
        //     "meshgraph/mesh",
        //     &rerun::Mesh3D::new(
        //         buffers
        //             .positions
        //             .into_iter()
        //             .zip(buffers.normals.iter().cloned())
        //             .map(|(pos, norm)| vec3_array(pos - norm * 0.1)),
        //     )
        //     .with_triangle_indices(
        //         buffers
        //             .indices
        //             .chunks(3)
        //             .map(|chunk| rerun::datatypes::UVec3D::new(chunk[0], chunk[1], chunk[2])),
        //     )
        //     .with_vertex_colors(
        //         buffers
        //             .normals
        //             .into_iter()
        //             .map(|v| {
        //                 [
        //                     (100.0 + v.x * 100.0) as u8,
        //                     (100.0 + v.y * 100.0) as u8,
        //                     (100.0 + v.z * 100.0) as u8,
        //                 ]
        //             })
        //             .collect::<Vec<_>>(),
        //     ),
        // )
        // .unwrap();

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
            use itertools::Itertools;

            let pos = face.vertices(self).map(|v| self.positions[v]).collect_vec();

            let center = pos.iter().copied().reduce(|a, b| a + b).unwrap() / pos.len() as f32;

            let pos = face
                .vertices(self)
                .zip(pos)
                .map(|(v, p)| (v, center * 0.1 + p * 0.9))
                .collect::<HashMap<_, _>>();

            for he_id in face.halfedges(self) {
                let he = self.halfedges[he_id];

                let start = pos[&he.start_vertex(self).unwrap()];
                let end = pos[&he.end_vertex];

                he_to_pos.insert(he_id, (start, end));

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        for (he_id, he) in &self.halfedges {
            if he.is_boundary() {
                let start_vertex = he.start_vertex(self).unwrap();
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
            let twin = he.twin.unwrap();

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
