use hashbrown::{HashMap, HashSet};
use itertools::Itertools;

use crate::{FaceId, HalfedgeId, MeshGraph, Selection, SelectionOps, VertexId};

impl MeshGraph {
    /// Collapses edges until all edges have a length above the minimum length.
    ///
    /// This will schedule necessary updates to the QBVH but you have to call
    /// `refit()` and maybe `rebalance()` after the operation.
    pub fn collapse_until_edges_above_min_length(
        &mut self,
        min_length_squared: f32,
        selection: &mut Selection,
    ) {
        let mut dedup_halfedges = HashSet::new();

        for he in selection.resolve_to_halfedges(self) {
            let twin = self.halfedges[he].twin;
            let twin_already_in = twin
                .map(|twin| dedup_halfedges.contains(&twin))
                .unwrap_or_default();

            if !twin_already_in {
                dedup_halfedges.insert(he);
            }
        }

        let mut halfedges_to_collapse = dedup_halfedges
            .into_iter()
            .filter_map(|he| {
                let len = self.halfedges[he].length_squared(self);
                (len < min_length_squared).then_some((he, len))
            })
            .collect::<HashMap<_, _>>();

        while !halfedges_to_collapse.is_empty() {
            let mut min_len = f32::MAX;
            let mut min_he = HalfedgeId::default();

            for (&he, &len) in &halfedges_to_collapse {
                if len < min_len {
                    min_len = len;
                    min_he = he;
                }
            }

            let start_vertex = self.halfedges[min_he].start_vertex(self);

            let (verts, halfedges, faces) = self.collapse_edge(min_he);

            #[cfg(feature = "rerun")]
            {
                crate::RR
                    .log("meshgraph/face/disolve", &rerun::Clear::recursive())
                    .unwrap();
                crate::RR
                    .log("meshgraph/halfedge/disolve", &rerun::Clear::recursive())
                    .unwrap();
                crate::RR
                    .log("meshgraph/vertex/disolve", &rerun::Clear::recursive())
                    .unwrap();
                self.log_rerun();
            }

            halfedges_to_collapse.remove(&min_he);

            for vert in verts {
                selection.remove(vert);
            }
            for halfedge in halfedges {
                selection.remove(halfedge);
                halfedges_to_collapse.remove(&halfedge);
            }
            for face in faces {
                selection.remove(face);
            }

            for halfedge_id in self.vertices[start_vertex].outgoing_halfedges(self) {
                let halfedge = &self.halfedges[halfedge_id];

                let len = halfedge.length_squared(self);

                if len < min_length_squared {
                    halfedges_to_collapse.insert(halfedge_id, len);
                } else {
                    halfedges_to_collapse.remove(&halfedge_id);
                }

                if let Some(twin) = halfedge.twin {
                    halfedges_to_collapse.remove(&twin);
                }

                if let Some(face) = halfedge.face {
                    self.qbvh.pre_update_or_insert(self.faces[face]);
                }
                selection.insert(halfedge_id);
            }

            #[cfg(feature = "rerun")]
            {
                self.log_rerun();
            }
        }
    }

    /// Disolve an edge in the mesh graph.
    ///
    /// This moves the start vertex of the edge to the center of the edge
    /// and removes the end vertex and the adjacent and opposite faces.
    /// Returns the vertex, halfedges and faces that were removed.
    pub fn collapse_edge(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> (Vec<VertexId>, Vec<HalfedgeId>, Vec<FaceId>) {
        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("disolve/he", halfedge_id);
        }

        // TODO : consider border vertices

        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        let he = self.halfedges[halfedge_id];

        let start_vert = he.start_vertex(self);
        let end_vert = he.end_vertex;

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("disolve/remove_disolved", halfedge_id);
            self.log_he_rerun("disolve/remove_twin", he.twin());
            self.log_vert_rerun("disolve/remove_end", end_vert);
        }

        let end_outgoing_halfedges = self.vertices[end_vert].outgoing_halfedges(self);
        let end_incoming_halfedges = self.vertices[end_vert].incoming_halfedges(self);

        let center = (self.positions[start_vert] + self.positions[end_vert]) * 0.5;

        let (face_id, halfedges) = self.remove_halfedge_face(halfedge_id);
        removed_faces.push(face_id);
        removed_halfedges.extend(halfedges);
        removed_halfedges.push(halfedge_id);

        let twin = he.twin();
        if !self.halfedges[twin].is_boundary() {
            let (face_id, halfedges) = self.remove_halfedge_face(twin);
            removed_faces.push(face_id);
            removed_halfedges.extend(halfedges);
        }
        removed_halfedges.push(twin);

        for end_incoming_he in end_incoming_halfedges {
            if let Some(end_incoming_he_mut) = self.halfedges.get_mut(end_incoming_he) {
                end_incoming_he_mut.end_vertex = start_vert;
            }
        }

        self.vertices.remove(end_vert);
        self.positions.remove(end_vert);
        if let Some(normals) = &mut self.vertex_normals {
            normals.remove(end_vert);
        }
        self.halfedges.remove(halfedge_id);
        self.halfedges.remove(twin);
        removed_halfedges.push(twin);

        self.positions[start_vert] = center;

        self.vertices[start_vert].outgoing_halfedge = end_outgoing_halfedges
            .into_iter()
            .find(|he_id| self.halfedges.contains_key(*he_id));

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun(
                "disolve/outgoing",
                self.vertices[start_vert].outgoing_halfedge.unwrap(),
            );
        }

        let mut removed_vertices = vec![end_vert];

        // Remove flaps (= faces that share all their vertices)
        let mut face_tuples: Vec<_> = self.vertices[start_vert]
            .faces(self)
            .into_iter()
            .circular_tuple_windows()
            .collect();

        while let Some((face_id1, face_id2)) = face_tuples.pop() {
            if self.faces_share_all_vertices(face_id1, face_id2) {
                let (vs, hes) = self.delete_face(face_id1);
                removed_vertices.extend(vs);
                removed_halfedges.extend(hes);
                removed_faces.push(face_id1);

                let (vs, hes) = self.delete_face(face_id2);
                removed_vertices.extend(vs);
                removed_halfedges.extend(hes);
                removed_faces.push(face_id2);

                // one of the faces in the next tuple was removed, so we need to remove it.
                face_tuples.pop();
            }
        }

        (removed_vertices, removed_halfedges, removed_faces)
    }

    /// Remove a halfedge face and re-connecting the adjacent halfedges.
    /// Only works on manifold triangle meshes.
    fn remove_halfedge_face(&mut self, halfedge_id: HalfedgeId) -> (FaceId, [HalfedgeId; 2]) {
        let he = self.halfedges[halfedge_id];

        let face_id = he
            .face
            .expect("only called from disolve_edge() when there is a face");

        let next_he_id = he.next.unwrap();
        let prev_he_id = he.prev(self).unwrap();

        let next_twin_id = self.halfedges[next_he_id].twin;
        let prev_twin_id = self.halfedges[prev_he_id].twin;

        let next_he = self.halfedges[next_he_id];
        let prev_he = self.halfedges[prev_he_id];

        let next_end_v_id = next_he.end_vertex;
        let prev_end_v_id = prev_he.end_vertex;

        self.vertices[next_end_v_id].outgoing_halfedge = next_he.next;
        self.vertices[prev_end_v_id].outgoing_halfedge = prev_he.next;

        let prev_start_v_id = prev_he.start_vertex(self);
        let prev_start_v = self.vertices[prev_start_v_id];

        if prev_start_v.outgoing_halfedge == Some(prev_he_id) {
            self.vertices[prev_start_v_id].outgoing_halfedge = prev_he
                .ccw_rotated_neighbour(self)
                .or_else(|| prev_he.cw_rotated_neighbour(self));
        }

        #[cfg(feature = "rerun")]
        {
            self.log_face_rerun("disolve/remove_face", face_id);
            self.log_he_rerun("disolve/remove_next", next_he_id);
            self.log_he_rerun("disolve/remove_prev", prev_he_id);
        }

        self.qbvh.remove(self.faces[face_id]);

        self.halfedges.remove(next_he_id);
        self.halfedges.remove(prev_he_id);
        self.faces.remove(face_id);

        if let Some(next_twin) = next_twin_id {
            self.halfedges[next_twin].twin = prev_twin_id;
        }

        if let Some(prev_twin) = prev_twin_id {
            self.halfedges[prev_twin].twin = next_twin_id;
        }

        (face_id, [next_he_id, prev_he_id])
    }
}
