use glam::Vec3;
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
                    .log("meshgraph/face/collapse", &rerun::Clear::recursive())
                    .unwrap();
                crate::RR
                    .log("meshgraph/halfedge/collapse", &rerun::Clear::recursive())
                    .unwrap();
                crate::RR
                    .log("meshgraph/vertex/collapse", &rerun::Clear::recursive())
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

    /// Collapse an edge in the mesh graph.
    ///
    /// This moves the start vertex of the edge to the center of the edge
    /// and removes the end vertex and the adjacent and opposite faces.
    ///
    /// It also performs a cleanup afterwards to remove flaps (faces that share the same vertices).
    ///
    /// Returns the vertices, halfedges and faces that were removed.
    pub fn collapse_edge(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> (Vec<VertexId>, Vec<HalfedgeId>, Vec<FaceId>) {
        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("collapse/he", halfedge_id);
        }

        // TODO : consider border vertices

        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        let he = self.halfedges[halfedge_id];

        let start_vert_id = he.start_vertex(self);
        let end_vert_id = he.end_vertex;

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("collapse/remove_collapsed", halfedge_id);
            self.log_he_rerun("collapse/remove_twin", he.twin());
            self.log_vert_rerun("collapse/remove_end", end_vert_id);
        }

        let end_outgoing_halfedges = self.vertices[end_vert_id].outgoing_halfedges(self);
        let end_incoming_halfedges = self.vertices[end_vert_id].incoming_halfedges(self);

        let start_pos = self.positions[start_vert_id];
        let end_pos = self.positions[end_vert_id];

        let center_pos = (start_pos + end_pos) * 0.5;

        let mut normal = None;
        if !he.is_boundary() {
            let third_pos = self.positions[self.halfedges[he.next.unwrap()].end_vertex];
            normal = Some(
                (start_pos - end_pos)
                    .cross(start_pos - third_pos)
                    .normalize(),
            );

            let (face_id, halfedges) = self.remove_halfedge_face(halfedge_id);
            removed_faces.push(face_id);
            removed_halfedges.extend(halfedges);
        }
        removed_halfedges.push(halfedge_id);

        let twin_id = he.twin();
        let twin = &self.halfedges[twin_id];
        if !twin.is_boundary() {
            if normal.is_none() {
                let third_pos = self.positions[self.halfedges[twin.next.unwrap()].end_vertex];
                normal = Some((end_pos - start_pos).cross(end_pos - third_pos).normalize());
            }

            let (face_id, halfedges) = self.remove_halfedge_face(twin_id);
            removed_faces.push(face_id);
            removed_halfedges.extend(halfedges);
        }
        removed_halfedges.push(twin_id);

        for end_incoming_he in end_incoming_halfedges {
            if let Some(end_incoming_he_mut) = self.halfedges.get_mut(end_incoming_he) {
                end_incoming_he_mut.end_vertex = start_vert_id;
            }
        }

        self.vertices.remove(end_vert_id);
        self.positions.remove(end_vert_id);
        if let Some(normals) = &mut self.vertex_normals {
            normals.remove(end_vert_id);
        }
        self.halfedges.remove(halfedge_id);
        self.halfedges.remove(twin_id);
        removed_halfedges.push(twin_id);

        self.positions[start_vert_id] = center_pos;

        self.vertices[start_vert_id].outgoing_halfedge = end_outgoing_halfedges
            .iter()
            .find(|he_id| self.halfedges.contains_key(**he_id))
            .copied();

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun(
                "collapse/outgoing",
                self.vertices[start_vert_id].outgoing_halfedge.unwrap(),
            );
        }

        let mut removed_vertices = vec![end_vert_id];

        // Remove flaps (= faces that share all their vertices)
        let mut face_tuples: Vec<_> = self.vertices[start_vert_id]
            .faces(self)
            .into_iter()
            .circular_tuple_windows()
            .collect();

        let mut first = true;

        while let Some((face_id1, face_id2)) = face_tuples.pop() {
            #[cfg(feature = "rerun")]
            {
                self.log_rerun();
                crate::RR.flush_blocking();
            }

            if self.faces_share_all_vertices(face_id1, face_id2) {
                #[cfg(feature = "rerun")]
                {
                    if self.vertices[start_vert_id].faces(self).len() == 1 {
                        self.log_vert_rerun("cleanup_delete", start_vert_id);
                    }

                    self.log_face_rerun("cleanup_delete/1", face_id1);
                    self.log_face_rerun("cleanup_delete/2", face_id2);
                    crate::RR.flush_blocking();
                }

                let mut halfedges_of_faces = HashSet::new();
                halfedges_of_faces.extend(self.faces[face_id1].halfedges(self));
                halfedges_of_faces.extend(self.faces[face_id2].halfedges(self));

                let (vs, hes) = self.delete_face(face_id1);
                for he_id in &hes {
                    halfedges_of_faces.remove(he_id);
                }
                removed_vertices.extend(vs);
                removed_halfedges.extend(hes);
                removed_faces.push(face_id1);

                let (vs, hes) = self.delete_face(face_id2);
                for he_id in &hes {
                    halfedges_of_faces.remove(he_id);
                }
                removed_vertices.extend(vs);
                removed_halfedges.extend(hes);
                removed_faces.push(face_id2);

                // the twin edges of the neighboring faces of the deleted faces are still there
                // we need to remove them and re-connect (twin) their twin edges
                debug_assert_eq!(halfedges_of_faces.len(), 2);
                let mut halfedges_of_faces = halfedges_of_faces.into_iter();
                let he_id1 = halfedges_of_faces.next().unwrap();
                let he_id2 = halfedges_of_faces.next().unwrap();

                let twin_id1 = self.halfedges[he_id1].twin();
                let twin_id2 = self.halfedges[he_id2].twin();
                self.halfedges[twin_id1].twin = Some(twin_id2);
                self.halfedges[twin_id2].twin = Some(twin_id1);

                self.halfedges.remove(he_id1);
                self.halfedges.remove(he_id2);
                removed_halfedges.push(he_id1);
                removed_halfedges.push(he_id2);

                let new_outgoing_he_id = if self.halfedges[twin_id1].end_vertex == start_vert_id {
                    twin_id2
                } else {
                    twin_id1
                };

                // Also update both vertices of the deleted halfedges
                self.vertices[start_vert_id].outgoing_halfedge = Some(new_outgoing_he_id);

                let new_outgoing_he = self.halfedges[new_outgoing_he_id];
                self.vertices[new_outgoing_he.end_vertex].outgoing_halfedge = new_outgoing_he.twin;

                // the next tuple contains one of the removed faces, so re remove it
                face_tuples.pop();

                // tuples wrap around so we need to remove the first element if the last element was removed
                if first {
                    face_tuples.remove(0);
                }
            }

            first = false;
        }

        self.check_vertex_faces_for_overlapping(start_vert_id, normal.unwrap());

        (removed_vertices, removed_halfedges, removed_faces)
    }

    /// Remove a halfedge face and re-connecting the adjacent halfedges.
    /// Only works on manifold triangle meshes.
    fn remove_halfedge_face(&mut self, halfedge_id: HalfedgeId) -> (FaceId, [HalfedgeId; 2]) {
        let he = self.halfedges[halfedge_id];

        let face_id = he
            .face
            .expect("only called from collapse_edge() when there is a face");

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
            self.log_face_rerun("collapse/remove_face", face_id);
            self.log_he_rerun("collapse/remove_next", next_he_id);
            self.log_he_rerun("collapse/remove_prev", prev_he_id);
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

    fn check_vertex_faces_for_overlapping(&mut self, vertex_id: VertexId, normal: Vec3) {
        let vertex = self.vertices[vertex_id];
        let pos = self.positions[vertex_id];

        let neighbours = vertex.neighbours(self);

        let mut prev_diff = (self.positions[neighbours[neighbours.len() - 1]] - pos).normalize();

        for neighbour_id in neighbours {
            let mut diff = (self.positions[neighbour_id] - pos).normalize();

            // smooth out degenerate faces
            if diff.cross(prev_diff).dot(normal) < 0.1 {
                let mut avg_pos = Vec3::ZERO;

                let n_neighbours = self.vertices[neighbour_id].neighbours(self);
                let len = n_neighbours.len();

                for n_id in n_neighbours {
                    avg_pos += self.positions[n_id];
                }
                avg_pos /= len as f32;

                self.positions[neighbour_id] = avg_pos;
                diff = (avg_pos - pos).normalize();
            }

            prev_diff = diff;
        }
    }
}
