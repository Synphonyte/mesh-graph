use glam::Vec3;
use hashbrown::{HashMap, HashSet};
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{
    FaceId, HalfedgeId, MeshGraph, Selection, SelectionOps, VertexId, error_none,
    utils::unwrap_or_return,
};

impl MeshGraph {
    /// Collapses edges until all edges have a length above the minimum length.
    ///
    /// This will schedule necessary updates to the BVH but you have to call
    /// `refit()` after the operation.
    #[instrument(skip(self))]
    pub fn collapse_until_edges_above_min_length(
        &mut self,
        min_length_squared: f32,
        selection: &mut Selection,
    ) {
        let mut dedup_halfedges = HashSet::new();

        for he in selection.resolve_to_halfedges(self) {
            let twin = self
                .halfedges
                .get(he)
                .or_else(error_none!("Halfedge not found"))
                .and_then(|he| he.twin.or_else(error_none!("Twin missing")));
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
            let mut min_he_id = None;

            for (&he_id, &len) in &halfedges_to_collapse {
                if len < min_len && self.can_collapse_edge(he_id, min_length_squared) {
                    min_len = len;
                    min_he_id = Some(he_id);
                }
            }

            if min_he_id.is_none() {
                // couldn't find a valid halfedge to collapse
                break;
            }
            let min_he_id = min_he_id.unwrap();

            let start_vertex = self.halfedges[min_he_id].start_vertex(self);

            let (verts, halfedges, faces) = self.collapse_edge(min_he_id);

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

            halfedges_to_collapse.remove(&min_he_id);

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

            if let Some(start_vertex) = start_vertex {
                let outgoing_halfedges = self
                    .vertices
                    .get(start_vertex)
                    .or_else(error_none!("Vertex for start vertex not found"))
                    .map(|vertex| vertex.outgoing_halfedges(self).collect::<Vec<_>>())
                    .unwrap_or_default();

                for halfedge_id in outgoing_halfedges {
                    if let Some(halfedge) = self.halfedges.get(halfedge_id) {
                        let len = halfedge.length_squared(self);

                        if len < min_length_squared {
                            halfedges_to_collapse.insert(halfedge_id, len);
                        } else {
                            halfedges_to_collapse.remove(&halfedge_id);
                        }

                        if let Some(twin) = halfedge.twin {
                            halfedges_to_collapse.remove(&twin);
                        }

                        if let Some(face_id) = halfedge.face {
                            if let Some(face) = self.faces.get(face_id) {
                                self.bvh.insert_or_update_partially(
                                    face.aabb(self),
                                    face.index,
                                    0.0,
                                );
                            } else {
                                error!("Face not found. BVH will not be updated.");
                            }
                        }
                        selection.insert(halfedge_id);
                    } else {
                        error!("Halfedge not found");
                    }
                }
            } else {
                error!("Start vertex not found")
            }

            #[cfg(feature = "rerun")]
            {
                self.log_rerun();
            }
        }
    }

    #[inline]
    fn can_collapse_edge(&mut self, halfedge_id: HalfedgeId, min_length_squared: f32) -> bool {
        self.can_collapse_edge_inner(halfedge_id, min_length_squared)
            .is_some()
    }

    #[instrument(skip(self))]
    fn can_collapse_edge_inner(
        &mut self,
        halfedge_id: HalfedgeId,
        min_length_squared: f32,
    ) -> Option<()> {
        // TODO : consider boundary edge
        //
        //          end_vertex
        //  .            .            .
        // ( ) ◀─────── ( ) ───────▶ ( )
        //  '      3     '     2      '
        //            ╱ ▲ │ ╲
        //          4╱  │ │  ╲1
        //          ╱   │ │0  ╲
        //         ╱    │ │    ╲
        //        ▼     │ │     ▼
        //       .      │ │      .
        //      ( )   he│ │twin ( )
        //       '      │ │      '
        //        ▲     │ │     ▲
        //         ╲    │ │    ╱
        //          ╲  0│ │   ╱
        //          1╲  │ │  ╱4
        //            ╲ │ ▼ ╱
        //  .      2     .     3      .
        // ( ) ◀─────── ( ) ───────▶ ( )
        //  '            '            '
        //         start_vertex

        let he = self
            .halfedges
            .get(halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;

        let start_vertex_id = he.start_vertex(self)?;
        self.vertices
            .get_mut(start_vertex_id)
            .or_else(error_none!("Start vertex not found"))?
            .outgoing_halfedge = Some(halfedge_id);

        // just made sure that this exists
        let start_outgoing_vertex_ids = self.vertices[start_vertex_id]
            .outgoing_halfedges(self)
            .skip(2)
            .filter_map(|he_id| {
                Some(
                    self.halfedges
                        .get(he_id)
                        .or_else(error_none!("Halfedge not found"))?
                        .end_vertex,
                )
            })
            .collect_vec();

        let twin_id = he.twin.or_else(error_none!("Twin halfedge not found"))?;

        let end_vertex_id = he.end_vertex;
        self.vertices
            .get_mut(end_vertex_id)
            .or_else(error_none!("End vertex not found"))?
            .outgoing_halfedge = Some(twin_id);

        // just made sure that this exists
        let end_outgoing_vertex_ids = self.vertices[end_vertex_id]
            .outgoing_halfedges(self)
            .skip(2)
            .filter_map(|he_id| {
                Some(
                    self.halfedges
                        .get(he_id)
                        .or_else(error_none!("Halfedge not found"))?
                        .end_vertex,
                )
            })
            .collect_vec();

        let neighbours = start_outgoing_vertex_ids
            .into_iter()
            .chain(end_outgoing_vertex_ids)
            .collect_vec();

        if neighbours.is_empty() {
            error!("Vertex has no neighbours");
            return None;
        }

        let start_pos = self
            .positions
            .get(start_vertex_id)
            .or_else(error_none!("Start position not found"))?;

        let end_pos = self
            .positions
            .get(end_vertex_id)
            .or_else(error_none!("End position not found"))?;

        let center = (start_pos + end_pos) * 0.5;

        let mut prev_diff = (self
            .positions
            .get(neighbours[neighbours.len() - 1])
            .or_else(error_none!("Last neighbour position not found"))?
            - center)
            .normalize();

        let mut normal = Vec3::ZERO;

        if let Some(face_id) = he.face {
            let face = self
                .faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?;
            normal += face.normal(self)?;
        }

        if let Some(face_id) = self
            .halfedges
            .get(twin_id)
            .or_else(error_none!("Face not found"))?
            .face
        {
            let face = self
                .faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?;
            normal += face.normal(self)?;
        }

        if normal == Vec3::ZERO {
            error!("Halfedge as no adjacent faces");
            return None;
        }

        normal = normal.normalize();

        // give a bit of leeway to the length of the neighbour edge
        let max_length_squared = min_length_squared * 5.0;

        for neighbour_id in neighbours {
            let diff = self
                .positions
                .get(neighbour_id)
                .or_else(error_none!("Neighbour position not found"))?
                - center;

            if diff.length_squared() > max_length_squared {
                // neighbour edge would be too long
                return None;
            }

            if diff.normalize().cross(prev_diff).dot(normal) < 0.1 {
                // degenerate or reversed triangle
                return None;
            }

            prev_diff = diff;
        }

        Some(())
    }

    /// Collapse an edge in the mesh graph.
    ///
    /// This moves the start vertex of the edge to the center of the edge
    /// and removes the end vertex and the adjacent and opposite faces.
    ///
    /// It also performs a cleanup afterwards to remove flaps (faces that share the same vertices).
    ///
    /// Returns the vertices, halfedges and faces that were removed.
    #[instrument(skip(self))]
    pub fn collapse_edge(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> (Vec<VertexId>, Vec<HalfedgeId>, Vec<FaceId>) {
        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("collapse/he", halfedge_id);
        }

        // TODO : consider border vertices

        let mut removed_vertices = Vec::new();
        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        let he = *unwrap_or_return!(
            self.halfedges.get(halfedge_id),
            "Halfedge not found",
            (removed_vertices, removed_halfedges, removed_faces)
        );

        let start_vert_id = unwrap_or_return!(
            he.start_vertex(self),
            "Start vertex not found",
            (removed_vertices, removed_halfedges, removed_faces)
        );
        let end_vert_id = he.end_vertex;

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("collapse/remove_collapsed", halfedge_id);
            if let Some(twin) = he.twin {
                self.log_he_rerun("collapse/remove_twin", twin);
            }
            self.log_vert_rerun("collapse/remove_end", end_vert_id);
        }

        let end_outgoing_halfedges = self.vertices[end_vert_id]
            .outgoing_halfedges(self)
            .collect::<Vec<_>>();
        let end_incoming_halfedges = self.vertices[end_vert_id]
            .incoming_halfedges(self)
            .collect::<Vec<_>>();

        let start_pos = *unwrap_or_return!(
            self.positions.get(start_vert_id),
            "Start position not found",
            (removed_vertices, removed_halfedges, removed_faces)
        );
        let end_pos = *unwrap_or_return!(
            self.positions.get(end_vert_id),
            "End position not found",
            (removed_vertices, removed_halfedges, removed_faces)
        );

        let center_pos = (start_pos + end_pos) * 0.5;

        if !he.is_boundary() {
            let (face_id, halfedges) = unwrap_or_return!(
                self.remove_halfedge_face(halfedge_id),
                "Could not remove face",
                (removed_vertices, removed_halfedges, removed_faces)
            );
            removed_faces.push(face_id);
            removed_halfedges.extend(halfedges);
        }
        removed_halfedges.push(halfedge_id);

        let twin_id = unwrap_or_return!(
            he.twin,
            "Twin missing",
            (removed_vertices, removed_halfedges, removed_faces)
        );
        let twin = unwrap_or_return!(
            self.halfedges.get(twin_id),
            "Halfedge not found",
            (removed_vertices, removed_halfedges, removed_faces)
        );

        if !twin.is_boundary() {
            let (face_id, halfedges) = unwrap_or_return!(
                self.remove_halfedge_face(twin_id),
                "Failed to remove halfedge face",
                (removed_vertices, removed_halfedges, removed_faces)
            );

            removed_faces.push(face_id);
            removed_halfedges.extend(halfedges);
        }
        removed_halfedges.push(twin_id);

        for end_incoming_he in end_incoming_halfedges {
            if let Some(end_incoming_he_mut) = self.halfedges.get_mut(end_incoming_he) {
                end_incoming_he_mut.end_vertex = start_vert_id;
            }
        }

        self.delete_only_vertex(end_vert_id);

        self.halfedges.remove(halfedge_id);
        self.halfedges.remove(twin_id);
        removed_halfedges.push(twin_id);

        self.positions[start_vert_id] = center_pos;

        self.vertices[start_vert_id].outgoing_halfedge = end_outgoing_halfedges
            .into_iter()
            .find(|he_id| self.halfedges.contains_key(*he_id))
            .or_else(error_none!("No new outgoing halfedge found"));

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun(
                "collapse/outgoing",
                self.vertices[start_vert_id].outgoing_halfedge.unwrap(),
            );
        }

        removed_vertices.push(end_vert_id);

        // Remove flaps (= faces that share all their vertices)
        let mut face_tuples: Vec<_> = unwrap_or_return!(
            self.vertices.get(start_vert_id),
            "Start vertex not found",
            (removed_vertices, removed_halfedges, removed_faces)
        )
        .faces(self)
        .collect::<Vec<_>>()
        .into_iter()
        .circular_tuple_windows()
        .collect();

        let mut first = true;

        while let Some((face_id1, face_id2)) = face_tuples.pop() {
            #[cfg(feature = "rerun")]
            {
                self.log_rerun();
            }

            if self.faces_share_all_vertices(face_id1, face_id2) {
                #[cfg(feature = "rerun")]
                {
                    if self.vertices[start_vert_id].faces(self).count() == 1 {
                        self.log_vert_rerun("cleanup_delete", start_vert_id);
                    }

                    self.log_face_rerun("cleanup_delete/1", face_id1);
                    self.log_face_rerun("cleanup_delete/2", face_id2);
                }

                let mut halfedges_of_faces = HashSet::new();
                // face existence already checked by `self.faces_share_all_vertices`
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

                let he1 = unwrap_or_return!(
                    self.halfedges.get(he_id1),
                    "Halfedge 1 missing",
                    (removed_vertices, removed_halfedges, removed_faces)
                );
                let twin_id1 = unwrap_or_return!(
                    he1.twin,
                    "Twin 1 missing",
                    (removed_vertices, removed_halfedges, removed_faces)
                );
                let he2 = unwrap_or_return!(
                    self.halfedges.get(he_id2),
                    "Halfedge 2 missing",
                    (removed_vertices, removed_halfedges, removed_faces)
                );
                let twin_id2 = unwrap_or_return!(
                    he2.twin,
                    "Twin 2 missing",
                    (removed_vertices, removed_halfedges, removed_faces)
                );

                {
                    let twin1 = unwrap_or_return!(
                        self.halfedges.get_mut(twin_id1),
                        "Twin 1 missing",
                        (removed_vertices, removed_halfedges, removed_faces)
                    );
                    twin1.twin = Some(twin_id2);
                }

                {
                    let twin2 = unwrap_or_return!(
                        self.halfedges.get_mut(twin_id2),
                        "Twin 2 missing",
                        (removed_vertices, removed_halfedges, removed_faces)
                    );
                    twin2.twin = Some(twin_id1);
                }

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
                unwrap_or_return!(
                    self.vertices.get_mut(start_vert_id),
                    "Start vertex not found",
                    (removed_vertices, removed_halfedges, removed_faces)
                )
                .outgoing_halfedge = Some(new_outgoing_he_id);

                let new_outgoing_he = unwrap_or_return!(
                    self.halfedges.get(new_outgoing_he_id),
                    "New outgoing halfedge not found",
                    (removed_vertices, removed_halfedges, removed_faces)
                );
                unwrap_or_return!(
                    self.vertices.get_mut(new_outgoing_he.end_vertex),
                    "End vertex not found",
                    (removed_vertices, removed_halfedges, removed_faces)
                )
                .outgoing_halfedge = new_outgoing_he
                    .twin
                    .or_else(error_none!("New outgoing twin missing"));

                // the next tuple contains one of the removed faces, so re remove it
                face_tuples.pop();

                // tuples wrap around so we need to remove the first element if the last element was removed
                if first {
                    face_tuples.remove(0);
                }
            }

            first = false;
        }

        (removed_vertices, removed_halfedges, removed_faces)
    }

    /// Remove a halfedge face and re-connecting the adjacent halfedges.
    /// Only works on manifold triangle meshes.
    #[instrument(skip(self))]
    fn remove_halfedge_face(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> Option<(FaceId, [HalfedgeId; 2])> {
        let he = self
            .halfedges
            .get(halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;

        let face_id = he.face.or_else(error_none!("Face not found"))?;

        let next_he_id = he.next.or_else(error_none!("Next halfedge is None"))?;
        let prev_he_id = he
            .prev(self)
            .or_else(error_none!("Previous halfedge is None"))?;

        let next_twin_id = self.halfedges[next_he_id]
            .twin
            .or_else(error_none!("Next twin halfedge not found"))?;
        let prev_twin_id = self.halfedges[prev_he_id]
            .twin
            .or_else(error_none!("Previous twin halfedge not found"))?;

        let next_he = self
            .halfedges
            .get(next_he_id)
            .or_else(error_none!("Next halfedge not found"))?;
        let prev_he = self
            .halfedges
            .get(prev_he_id)
            .or_else(error_none!("Previous halfedge not found"))?;

        let next_end_v_id = next_he.end_vertex;
        let prev_end_v_id = prev_he.end_vertex;

        self.vertices
            .get_mut(next_end_v_id)
            .or_else(error_none!("Next end vertex not found"))?
            .outgoing_halfedge = next_he.next.or_else(error_none!("Next next is None"));
        self.vertices
            .get_mut(prev_end_v_id)
            .or_else(error_none!("Previous end vertex not found"))?
            .outgoing_halfedge = prev_he.next.or_else(error_none!("Previous next is None"));

        let prev_start_v_id = prev_he
            .start_vertex(self)
            .or_else(error_none!("Previous start vertex ID not found"))?;
        let prev_start_v = self
            .vertices
            .get(prev_start_v_id)
            .or_else(error_none!("Previous start vertex not found"))?;

        if prev_start_v.outgoing_halfedge == Some(prev_he_id) {
            self.vertices[prev_start_v_id].outgoing_halfedge = prev_he
                .ccw_rotated_neighbour(self)
                .or_else(|| prev_he.cw_rotated_neighbour(self))
                .or_else(error_none!(
                    "Previous start vertex new outgoing halfedge not found"
                ));
        }

        #[cfg(feature = "rerun")]
        {
            self.log_face_rerun("collapse/remove_face", face_id);
            self.log_he_rerun("collapse/remove_next", next_he_id);
            self.log_he_rerun("collapse/remove_prev", prev_he_id);
        }

        self.bvh.remove(
            self.faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?
                .index,
        );

        self.halfedges.remove(next_he_id);
        self.halfedges.remove(prev_he_id);
        self.faces.remove(face_id);

        self.halfedges
            .get_mut(next_twin_id)
            .or_else(error_none!("Next twin halfedge not found"))?
            .twin = Some(prev_twin_id);

        self.halfedges
            .get_mut(prev_twin_id)
            .or_else(error_none!("Previous twin halfedge not found"))?
            .twin = Some(next_twin_id);

        Some((face_id, [next_he_id, prev_he_id]))
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_collapse_edge() {
        let mut mesh_graph = MeshGraph::new();

        let face1 = mesh_graph
            .create_face_from_positions(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 4.0, 0.0),
                Vec3::new(1.0, 2.0, 0.0),
            )
            .face_id;

        let he1 = mesh_graph.faces[face1]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[1];

        let face2 = mesh_graph
            .create_face_from_halfedge_and_position(he1, Vec3::new(2.0, 4.0, 0.0))
            .unwrap()
            .face_id;

        let he2 = mesh_graph.faces[face2]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[2];

        let face3 = mesh_graph
            .create_face_from_halfedge_and_position(he2, Vec3::new(3.0, 2.0, 0.0))
            .unwrap()
            .face_id;

        let he3 = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[1];

        let face4 = mesh_graph
            .create_face_from_halfedge_and_position(he3, Vec3::new(4.0, 4.0, 0.0))
            .unwrap()
            .face_id;

        let he4 = mesh_graph.faces[face4]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face5 = mesh_graph
            .create_face_from_halfedge_and_position(he4, Vec3::new(4.0, 0.0, 0.0))
            .unwrap()
            .face_id;

        let he5 = mesh_graph.faces[face5]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face6 = mesh_graph
            .create_face_from_halfedge_and_position(he5, Vec3::new(2.0, 0.0, 0.0))
            .unwrap()
            .face_id;

        let he6 = mesh_graph.faces[face6]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let he3 = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face7 = mesh_graph
            .create_face_from_halfedges(he6, he3)
            .unwrap()
            .face_id;

        let he7 = mesh_graph.faces[face7]
            .halfedges(&mesh_graph)
            .next()
            .unwrap();

        let he1 = mesh_graph.faces[face1]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face8 = mesh_graph
            .create_face_from_halfedges(he1, he7)
            .unwrap()
            .face_id;

        #[cfg(feature = "rerun")]
        let faces = vec![face1, face2, face3, face4, face5, face6, face7, face8];

        assert_eq!(mesh_graph.vertices.len(), 8);
        assert_eq!(mesh_graph.halfedges.len(), 30);
        assert_eq!(mesh_graph.faces.len(), 8);

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_faces_rerun_with_name("test_edge_collapse".into(), &faces);
            crate::RR.flush_blocking().unwrap();
        }

        let edge_to_collapse = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let (removed_vertex_ids, removed_halfedge_ids, removed_face_ids) =
            mesh_graph.collapse_edge(edge_to_collapse);

        assert_eq!(removed_vertex_ids.len(), 1);
        assert_eq!(removed_halfedge_ids.len(), 7);
        assert_eq!(removed_face_ids.len(), 2);

        assert_eq!(mesh_graph.vertices.len(), 7);
        assert_eq!(mesh_graph.halfedges.len(), 24);
        assert_eq!(mesh_graph.faces.len(), 6);

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_faces_rerun_with_name(
                "test_edge_collapse".into(),
                &faces
                    .into_iter()
                    .filter(|face| !removed_face_ids.contains(face))
                    .collect_vec(),
            );
            crate::RR.flush_blocking().unwrap();
        }
    }
}
