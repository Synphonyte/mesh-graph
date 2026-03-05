use glam::Vec3;
use hashbrown::HashMap;
use itertools::Itertools;
use tracing::error;

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

    pub fn log_verts_rerun(&self, name: &str, vertices: &[VertexId]) {
        RR.log(
            format!("meshgraph/vertex/{name}"),
            &rerun::Points3D::new(
                vertices
                    .iter()
                    .map(|&v_id| vec3_array(self.positions[v_id])),
            ),
        )
        .unwrap();
    }

    pub fn log_verts_w_labels_rerun(
        &self,
        name: &str,
        vertices: &[VertexId],
        labels: &[impl AsRef<str>],
    ) {
        RR.log(
            format!("meshgraph/vertex/{name}"),
            &rerun::Points3D::new(
                vertices
                    .iter()
                    .map(|&v_id| vec3_array(self.positions[v_id])),
            )
            .with_labels(labels.iter().map(|l| l.as_ref())),
        )
        .unwrap();
    }

    pub fn log_he_rerun(&self, name: &str, halfedge: HalfedgeId) {
        use crate::RR;
        use crate::utils::*;

        let he = self.halfedges[halfedge];

        let start = unwrap_or_return!(
            self.positions.get(he.start_vertex(self).unwrap()),
            "Start vertex of he {halfedge:?} not found"
        );
        let end = unwrap_or_return!(
            self.positions.get(he.end_vertex),
            "End vertex of he {halfedge:?} not found"
        );

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

            let Some(start) = self.positions.get(he.start_vertex(self).unwrap()) else {
                error!(
                    "Missing position for start vertex {:?} of he {:?}",
                    he.start_vertex(self).unwrap(),
                    he_id
                );
                continue;
            };
            let Some(end) = self.positions.get(he.end_vertex) else {
                error!(
                    "Missing position for end vertex {:?} of he {:?}",
                    he.end_vertex, he_id
                );
                continue;
            };

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

    pub fn log_face_rerun(&self, name: &str, face_id: FaceId) {
        let mut origins = Vec::with_capacity(3);
        let mut vectors = Vec::with_capacity(3);

        let face = self.faces[face_id];

        let pos = face
            .vertices(self)
            .filter_map(|v| self.positions.get(v).copied())
            .collect_vec();

        if pos.len() < 3 {
            error!("face {face:?} has less than 3 vertices or positions");
            return;
        }

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
        let (vert_labels, vert_pos): (Vec<_>, Vec<_>) = self
            .positions
            .iter()
            .map(|(id, pos)| (v_label(id), vec3_array(pos)))
            .unzip();

        RR.log(
            "meshgraph/positions",
            &rerun::Points3D::new(vert_pos)
                .with_radii(self.positions.iter().map(|_| 0.01))
                .with_labels(vert_labels),
        )
        .unwrap();

        let mut origins = Vec::with_capacity(self.faces.len() * 3);
        let mut vectors = Vec::with_capacity(self.faces.len() * 3);
        let mut labels = Vec::<String>::with_capacity(self.faces.len() * 3);

        let mut he_to_pos = HashMap::<HalfedgeId, (Vec3, Vec3)>::default();
        let mut face_to_center = HashMap::<FaceId, Vec3>::default();

        for face in self.faces.values() {
            let mut pos = HashMap::new();

            let mut center = Vec3::ZERO;

            for (he_id, he) in &self.halfedges {
                if he.face != Some(face.id) {
                    continue;
                }

                let Some(end_pos) = self.positions.get(he.end_vertex) else {
                    error!(
                        "Missing position for end vertex {:?} of he {:?}",
                        he.end_vertex, he_id
                    );
                    continue;
                };
                pos.insert(he.end_vertex, *end_pos);

                center += end_pos;
            }

            center /= pos.len().max(1) as f32;

            face_to_center.insert(face.id, center);

            for p in pos.values_mut() {
                *p = center * 0.1 + *p * 0.9;
            }

            for (he_id, he) in self.halfedges.iter() {
                if he.face != Some(face.id) {
                    continue;
                }

                let start_v_id = he.start_vertex(self).unwrap();
                let Some(start) = pos
                    .get(&start_v_id)
                    .copied()
                    .or_else(|| self.positions.get(start_v_id).copied())
                else {
                    error!(
                        "Missing position for start vertex {:?} of he {:?}",
                        start_v_id, he_id
                    );
                    continue;
                };
                let Some(end) = pos.get(&he.end_vertex).copied() else {
                    error!(
                        "Missing position for end vertex {:?} of he {:?}",
                        he.end_vertex, he_id
                    );
                    continue;
                };

                he_to_pos.insert(he_id, (start, end));

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
                labels.push(he_label(he_id));
            }
        }

        for (he_id, he) in &self.halfedges {
            if he.is_boundary() {
                let start_vertex = he.start_vertex(self).unwrap();
                let end_vertex = he.end_vertex;

                let Some(&start) = self.positions.get(start_vertex) else {
                    error!(
                        "Missing position for start vertex {:?} of he {:?}",
                        start_vertex, he_id
                    );
                    continue;
                };
                let Some(&end) = self.positions.get(end_vertex) else {
                    error!(
                        "Missing position for end vertex {:?} of he {:?}",
                        end_vertex, he_id
                    );
                    continue;
                };

                let offset = if let Some(normals) = self.vertex_normals.as_ref()
                    && let Some(start_normal) = normals.get(start_vertex)
                    && let Some(end_normal) = normals.get(end_vertex)
                {
                    start_normal
                        .lerp(*end_normal, 0.5)
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
                labels.push(he_label(he_id));
            }
        }

        RR.log(
            "meshgraph/halfedges",
            &rerun::Arrows3D::from_vectors(&vectors)
                .with_origins(&origins)
                .with_labels(labels.clone()),
        )
        .unwrap();

        origins.clear();
        vectors.clear();
        labels.clear();

        for (he_id, he) in &self.halfedges {
            let twin = he.twin.unwrap();

            let Some(&(he_start, he_end)) = he_to_pos.get(&he_id) else {
                error!("Missing position for halfedge {:?}", he_id);
                continue;
            };
            let Some(&(tw_start, tw_end)) = he_to_pos.get(&twin) else {
                error!("Missing position for twin halfedge {:?}", twin);
                continue;
            };

            let start = he_start * 0.8 + he_end * 0.2;
            let end = tw_start * 0.2 + tw_end * 0.8;

            origins.push(vec3_array(start));
            vectors.push(vec3_array(end - start));
        }

        RR.log(
            "meshgraph/halfedges/twins",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for (v_id, v) in &self.vertices {
            if let Some(he) = v.outgoing_halfedge.as_ref() {
                let start = self.positions[v_id];

                RR.log(
                    "meshgraph/vertices",
                    &rerun::Points3D::new([vec3_array(start)]),
                )
                .unwrap();

                let Some(&(start_he, end_he)) = he_to_pos.get(he) else {
                    error!("Halfedge {he:?} not found in mapped positions");
                    continue;
                };

                let end = start_he.lerp(end_he, 0.05);

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            "meshgraph/vertices/outgoing_halfedge",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();
        labels.clear();

        for v_id in self.vertices.keys() {
            for he in self
                .outgoing_halfedges
                .get(v_id)
                .cloned()
                .unwrap_or_default()
            {
                let start = self.positions[v_id];

                let Some(&(start_he, end_he)) = he_to_pos.get(&he) else {
                    error!("Halfedge {he:?} not found in mapped halfedges");
                    continue;
                };

                let end = start_he.lerp(end_he, 0.1);

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            "meshgraph/vertices/outgoing_halfedges",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();
        labels.clear();

        for face in self.faces.values() {
            let start = face_to_center[&face.id];

            let Some(&(he_start, he_end)) = he_to_pos.get(&face.halfedge) else {
                error!("Halfedge {:?} not found in mapped halfedges", face.halfedge);
                continue;
            };

            let end = he_start * 0.6 + he_end * 0.4;

            origins.push(vec3_array(start));
            vectors.push(vec3_array(end - start));
            labels.push(face_label(face.id));
        }

        RR.log(
            "meshgraph/face_halfedges",
            &rerun::Arrows3D::from_vectors(&vectors)
                .with_origins(&origins)
                .with_labels(labels.clone()),
        )
        .unwrap();

        origins.clear();
        vectors.clear();
        labels.clear();

        for (he_id, he) in &self.halfedges {
            if let Some(face_id) = he.face {
                let Some(&(he_start, he_end)) = he_to_pos.get(&he_id) else {
                    error!("Halfedge {he_id:?} not found in mapped halfedges");
                    continue;
                };
                let start = he_start * 0.4 + he_end * 0.6;

                let end = face_to_center[&face_id];

                origins.push(vec3_array(start));
                vectors.push(vec3_array((end - start) * 0.9));
            }
        }

        RR.log(
            "meshgraph/halfedges/faces",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

        origins.clear();
        vectors.clear();

        for (he_id, he) in &self.halfedges {
            if let Some(next_id) = he.next {
                let Some((he_start, he_end)) = he_to_pos.get(&he_id) else {
                    error!("Halfedge {he_id:?} not found in mapped halfedges");
                    continue;
                };
                let start = he_start * 0.15 + he_end * 0.85;

                let Some((next_start, next_end)) = he_to_pos.get(&next_id) else {
                    error!("Halfedge {next_id:?} not found in mapped halfedges");
                    continue;
                };
                let end = next_start * 0.85 + next_end * 0.15;

                origins.push(vec3_array(start));
                vectors.push(vec3_array(end - start));
            }
        }

        RR.log(
            "meshgraph/halfedges/next",
            &rerun::Arrows3D::from_vectors(&vectors).with_origins(&origins),
        )
        .unwrap();

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
    }
}

fn he_label(he_id: HalfedgeId) -> String {
    let label = format!("{he_id:?}");

    label["HalfedgeId(".len()..(label.len() - 1)].to_string()
}

fn v_label(v_id: VertexId) -> String {
    let label = format!("{v_id:?}");

    label["VertexId(".len()..(label.len() - 1)].to_string()
}

fn face_label(face_id: FaceId) -> String {
    let label = format!("{face_id:?}");

    label["FaceId(".len()..(label.len() - 1)].to_string()
}
