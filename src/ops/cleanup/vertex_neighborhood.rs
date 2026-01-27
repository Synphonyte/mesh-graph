use hashbrown::HashSet;
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

#[derive(Default)]
pub struct VertexNeighborhoodCleanup {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,
    /// Contains original vertex (first), new vertex (second) if a split was performed. Empty otherwise.
    pub split_vertices: Vec<VertexId>,
}

impl MeshGraph {
    /// Ensure that the neighborhood of a vertex is manifold.
    ///
    /// It removes flaps (two neighboring coincident triangles) and splits the given vertex
    /// in two if there are non-neighboring degenerate triangles or edges.
    ///
    /// See [Freestyle: Sculpting meshes with self-adaptive topology DOI 10.1016/j.cag.2011.03.033](https://inria.hal.science/inria-00606516v1/document)
    /// Chapters 3.2 and 5.1
    pub fn make_vertex_neighborhood_manifold(
        &mut self,
        vertex_id: VertexId,
    ) -> VertexNeighborhoodCleanup {
        let mut result = VertexNeighborhoodCleanup::default();
        result.split_vertices.push(vertex_id);

        let mut vertices = vec![vertex_id];

        loop {
            let (
                step_inserted_duplicated_vertex,
                step_removed_vertices,
                step_removed_halfedges,
                step_removed_faces,
            ) = self.make_vertex_neighborhood_manifold_step(vertices[0]);
            tracing::info!(
                "make manifold {step_inserted_duplicated_vertex:?} {step_removed_vertices:?} {step_removed_halfedges:?} {step_removed_faces:?}"
            );

            if let Some(inserted_vertex) = step_inserted_duplicated_vertex {
                vertices.push(inserted_vertex);
                result.split_vertices.push(inserted_vertex);
            }

            if step_removed_vertices.is_empty()
                && step_removed_halfedges.is_empty()
                && step_removed_faces.is_empty()
            {
                vertices.swap_remove(0);

                if vertices.is_empty() {
                    break;
                }
            }

            result.removed_vertices.extend(step_removed_vertices);
            result.removed_halfedges.extend(step_removed_halfedges);
            result.removed_faces.extend(step_removed_faces);
        }

        if result.split_vertices.len() == 1 {
            // nothing was split, just containing the original vertex id
            result.split_vertices.clear();
        }

        result
    }

    fn make_vertex_neighborhood_manifold_step(
        &mut self,
        vertex_id: VertexId,
    ) -> (
        Option<VertexId>,
        Vec<VertexId>,
        Vec<HalfedgeId>,
        Vec<FaceId>,
    ) {
        let mut removed_vertices = Vec::new();
        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        let inserted_duplicated_vertex = self.make_vertex_neighborhood_manifold_inner(
            vertex_id,
            &mut removed_vertices,
            &mut removed_halfedges,
            &mut removed_faces,
        );

        (
            inserted_duplicated_vertex,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        )
    }

    pub fn make_vertex_neighborhood_manifold_inner(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<VertexId> {
        tracing::info!("make_vertex_neighborhood_manifold_inner");

        self.vertices.get(vertex_id)?;

        tracing::info!("after vertices");

        self.remove_neighboring_flaps(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        );

        tracing::info!("after flaps");

        if let Some(inserted_duplicated_vertex) = self.remove_degenerate_faces(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        ) {
            return Some(inserted_duplicated_vertex);
        }

        tracing::info!("after faces");

        self.remove_degenerate_edges(vertex_id)
    }

    fn remove_degenerate_edges(&mut self, vertex_id: VertexId) -> Option<VertexId> {
        let halfedges = self
            .vertices
            .get(vertex_id)
            .or_else(error_none!("Vertex not found"))?
            .outgoing_halfedges(self)
            .collect_vec();

        for (he_id1, he_id2) in halfedges.into_iter().tuple_combinations() {
            if self.halfedges_share_all_vertices(he_id1, he_id2) {
                let he1 = self.halfedges[he_id1];
                let twin_id1 = he1.twin.or_else(error_none!("Twin not found"))?;
                let face_id1 = self
                    .halfedges
                    .get(twin_id1)
                    .or_else(error_none!("Halfedge not found"))?
                    .face
                    .or_else(error_none!("Face not found"))?;
                let face_id2 = self
                    .halfedges
                    .get(he_id2)
                    .or_else(error_none!("Halfedge not found"))?
                    .face
                    .or_else(error_none!("Face not found"))?;

                let coincident_face_ids = self.vertices[vertex_id].faces(self).collect_vec();
                let mut start_idx = 0;

                for (idx, face_id) in coincident_face_ids.iter().enumerate() {
                    if *face_id == face_id1 {
                        start_idx = idx;
                        break;
                    }
                }

                let mut end_idx = start_idx;
                let mut side_one = vec![];

                loop {
                    let face_id = coincident_face_ids[end_idx];

                    side_one.push(face_id);

                    end_idx += 1;
                    end_idx %= coincident_face_ids.len();

                    if face_id == face_id2 {
                        break;
                    }
                }

                let mut side_two = vec![];

                while end_idx != start_idx {
                    let face_id = coincident_face_ids[end_idx];

                    side_two.push(face_id);

                    end_idx += 1;
                    end_idx %= coincident_face_ids.len();
                }

                return self.split_regions_at_edge(vertex_id, he1.end_vertex, side_one, side_two);
            }
        }

        None
    }

    #[instrument(skip(self))]
    fn split_regions_at_edge(
        &mut self,
        vertex_id: VertexId,
        other_vertex_id: VertexId,
        side_one: Vec<FaceId>,
        side_two: Vec<FaceId>,
    ) -> Option<VertexId> {
        if side_one.len() < 2 || side_two.len() < 2 {
            error!("Not enough halfedges to split");
            return None;
        }

        #[cfg(feature = "rerun")]
        {
            self.log_vert_rerun("split_regions_at_edge", vertex_id);
            self.log_faces_rerun("split_regions_at_edge/side_one", &side_one);
            self.log_faces_rerun("split_regions_at_edge/side_two", &side_two);
        }

        let new_vertex_id = self.insert_vertex(
            *self
                .positions
                .get(vertex_id)
                .or_else(error_none!("Vertex position not found"))?,
        );

        // TODO : Move vertices apart?

        // Updating vertices
        for face_id in &side_two {
            let face = self
                .faces
                .get(*face_id)
                .or_else(error_none!("Face not found"))?;

            for he_id in face.halfedges(self).collect_vec() {
                // already checked in iterator that this he exists
                let he = &mut self.halfedges[he_id];

                if he.end_vertex == vertex_id {
                    he.end_vertex = new_vertex_id;
                }
            }
        }

        self.weld_faces_at(
            vertex_id,
            other_vertex_id,
            side_one[side_one.len() - 1],
            side_one[0],
        );
        self.weld_faces_at(
            new_vertex_id,
            other_vertex_id,
            side_two[0],
            side_two[side_two.len() - 1],
        );

        self.outgoing_halfedges[vertex_id] =
            self.vertices[vertex_id].outgoing_halfedges(self).collect();
        self.outgoing_halfedges[new_vertex_id] = self.vertices[new_vertex_id]
            .outgoing_halfedges(self)
            .collect();

        Some(new_vertex_id)
    }

    pub fn remove_degenerate_faces(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<VertexId> {
        let faces = self
            .vertices
            .get(vertex_id)
            .or_else(error_none!("Vertex not found"))?
            .faces(self)
            .collect_vec();

        for (face_id1, face_id2) in faces.into_iter().tuple_combinations() {
            if self.faces_share_all_vertices(face_id1, face_id2) {
                let coincident_face_ids = self.vertices[vertex_id].faces(self).collect_vec();
                let mut start_idx = 0;

                for (idx, face_id) in coincident_face_ids.iter().enumerate() {
                    if *face_id == face_id1 {
                        start_idx = (idx + 1) % coincident_face_ids.len();
                        break;
                    }
                }

                let mut end_idx = start_idx;
                let mut side_one = vec![];

                loop {
                    let face_id = coincident_face_ids[end_idx];

                    end_idx += 1;
                    end_idx %= coincident_face_ids.len();

                    if face_id == face_id2 {
                        break;
                    }

                    side_one.push(face_id);
                }

                let mut side_two = vec![];

                while end_idx != start_idx {
                    let face_id = coincident_face_ids[end_idx];

                    side_two.push(face_id);

                    end_idx += 1;
                    end_idx %= coincident_face_ids.len();
                }

                side_two.pop();

                let new_vertex_id =
                    self.split_regions_at_vertex(vertex_id, side_one, side_two, removed_halfedges);

                let (del_v_ids, del_he_ids) = self.delete_face(face_id1);
                removed_vertices.extend(del_v_ids);
                removed_halfedges.extend(del_he_ids);
                removed_faces.push(face_id1);

                let (del_v_ids, del_he_ids) = self.delete_face(face_id2);
                removed_vertices.extend(del_v_ids);
                removed_halfedges.extend(del_he_ids);
                removed_faces.push(face_id2);

                #[cfg(feature = "rerun")]
                {
                    self.log_rerun();
                }

                return new_vertex_id;
            }
        }

        None
    }

    #[instrument(skip(self))]
    fn split_regions_at_vertex(
        &mut self,
        vertex_id: VertexId,
        side_one: Vec<FaceId>,
        side_two: Vec<FaceId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
    ) -> Option<VertexId> {
        if side_one.len() < 2 || side_two.len() < 2 {
            error!("Not enough halfedges to split");
            return None;
        }

        #[cfg(feature = "rerun")]
        {
            self.log_vert_rerun("split_regions_at_vertex", vertex_id);
            self.log_faces_rerun("split_regions_at_vertex/side_one", &side_one);
            self.log_faces_rerun("split_regions_at_vertex/side_two", &side_two);
        }

        let new_vertex_id = self.insert_vertex(
            *self
                .positions
                .get(vertex_id)
                .or_else(error_none!("Vertex position not found"))?,
        );

        // TODO : Move vertices apart?

        // Updating start vertex
        for face_id in &side_two {
            let face = self
                .faces
                .get(*face_id)
                .or_else(error_none!("Face not found"))?;

            for he_id in face.halfedges(self).collect_vec() {
                // already checked in iterator that this he exists
                let he = &mut self.halfedges[he_id];
                if he.end_vertex == vertex_id {
                    he.end_vertex = new_vertex_id;
                }
            }
        }

        self.weld_faces(vertex_id, side_one[side_one.len() - 1], side_one[0]);
        self.weld_faces(new_vertex_id, side_two[0], side_two[side_two.len() - 1]);

        self.outgoing_halfedges[vertex_id] =
            self.vertices[vertex_id].outgoing_halfedges(self).collect();
        self.outgoing_halfedges[new_vertex_id] = self.vertices[new_vertex_id]
            .outgoing_halfedges(self)
            .collect();

        Some(new_vertex_id)
    }

    /// This finds the common edge between the two faces and welds them together by connecting
    /// the two `twin` relationships. The reverse `twin` pointers of the previous twins are not changed.
    ///
    /// `start_vertex_id` is shared by `face_id1` and `face_id2`.
    fn weld_faces(
        &mut self,
        start_vertex_id: VertexId,
        face_id1: FaceId,
        face_id2: FaceId,
    ) -> Option<()> {
        // #[cfg(feature = "rerun")]
        // {
        //     self.log_face_rerun("face1", face_id1);
        //     self.log_face_rerun("face2", face_id2);
        // }

        let face1 = self
            .faces
            .get(face_id1)
            .or_else(error_none!("Face not found"))?;
        let face2 = self
            .faces
            .get(face_id2)
            .or_else(error_none!("Face not found"))?;

        let face1_vertices = face1.vertices(self).collect::<HashSet<_>>();
        let face2_vertices = face2.vertices(self).collect::<HashSet<_>>();

        let mut other_common_vertex_id = None;

        for &v_id in face1_vertices.intersection(&face2_vertices) {
            if v_id != start_vertex_id {
                other_common_vertex_id = Some(v_id);
                break;
            }
        }

        let other_common_vertex_id =
            other_common_vertex_id.or_else(error_none!("No other common vertex found"))?;

        self.weld_faces_at(start_vertex_id, other_common_vertex_id, face_id1, face_id2)
    }

    #[instrument(skip(self))]
    fn weld_faces_at(
        &mut self,
        start_vertex_id: VertexId,
        other_common_vertex_id: VertexId,
        face_id1: FaceId,
        face_id2: FaceId,
    ) -> Option<()> {
        let face1 = self
            .faces
            .get(face_id1)
            .or_else(error_none!("Face not found"))?;
        let face2 = self
            .faces
            .get(face_id2)
            .or_else(error_none!("Face not found"))?;

        let he1_id = face1
            .halfedge_between(start_vertex_id, other_common_vertex_id, self)
            .or_else(error_none!("Halfedge between vertices not found"))?;

        let he2_id = face2
            .halfedge_between(start_vertex_id, other_common_vertex_id, self)
            .or_else(error_none!("Halfedge between vertices not found"))?;

        self.halfedges[he1_id].twin = Some(he2_id);
        self.halfedges[he2_id].twin = Some(he1_id);

        let (start_out_he_id, other_out_he_id) =
            if self.halfedges[he1_id].end_vertex == start_vertex_id {
                (he2_id, he1_id)
            } else {
                (he1_id, he2_id)
            };

        self.vertices
            .get_mut(start_vertex_id)
            .or_else(error_none!("Start vertex not found"))?
            .outgoing_halfedge = Some(start_out_he_id);
        self.vertices
            .get_mut(other_common_vertex_id)
            .or_else(error_none!("Other vertex not found"))?
            .outgoing_halfedge = Some(other_out_he_id);

        Some(())
    }

    fn remove_neighboring_flaps(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<()> {
        let faces = self
            .vertices
            .get(vertex_id)
            .or_else(error_none!("Vertex not found"))?
            .faces(self)
            .collect_vec();

        tracing::info!("remove_neighboring_flaps: {faces:?}");

        if faces.len() < 2 {
            return Some(());
        }

        let mut face_tuples = faces.into_iter().circular_tuple_windows().collect_vec();

        let mut first = true;

        while let Some((face_id1, face_id2)) = face_tuples.pop() {
            #[cfg(feature = "rerun")]
            {
                self.log_rerun();
            }

            tracing::info!("({face_id1:?}, {face_id2:?}) ; tuples: {face_tuples:?}");

            if self.faces_share_all_vertices(face_id1, face_id2) {
                #[cfg(feature = "rerun")]
                {
                    if self.vertices[vertex_id].faces(self).count() == 1 {
                        self.log_vert_rerun("make_vertex_neighborhood_manifold", vertex_id);
                    }

                    self.log_face_rerun("make_vertex_neighborhood_manifold/1", face_id1);
                    self.log_face_rerun("make_vertex_neighborhood_manifold/2", face_id2);
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

                let he1 = self
                    .halfedges
                    .get(he_id1)
                    .or_else(error_none!("Halfedge 1 missing"))?;
                let twin_id1 = he1.twin.or_else(error_none!("Twin 1 missing"))?;
                let he2 = self
                    .halfedges
                    .get(he_id2)
                    .or_else(error_none!("Halfedge 2 missing"))?;
                let twin_id2 = he2.twin.or_else(error_none!("Twin 2 missing"))?;

                {
                    let twin1 = self
                        .halfedges
                        .get_mut(twin_id1)
                        .or_else(error_none!("Twin 1 missing"))?;
                    twin1.twin = Some(twin_id2);
                }

                {
                    let twin2 = self
                        .halfedges
                        .get_mut(twin_id2)
                        .or_else(error_none!("Twin 2 missing"))?;
                    twin2.twin = Some(twin_id1);
                }

                self.halfedges.remove(he_id1);
                self.halfedges.remove(he_id2);
                removed_halfedges.push(he_id1);
                removed_halfedges.push(he_id2);

                let new_outgoing_he_id = if self.halfedges[twin_id1].end_vertex == vertex_id {
                    twin_id2
                } else {
                    twin_id1
                };

                // Also update both vertices of the deleted halfedges
                self.vertices
                    .get_mut(vertex_id)
                    .or_else(error_none!("Start vertex not found"))?
                    .outgoing_halfedge = Some(new_outgoing_he_id);

                let new_outgoing_he = self
                    .halfedges
                    .get(new_outgoing_he_id)
                    .or_else(error_none!("New outgoing halfedge not found"))?;

                self.vertices
                    .get_mut(new_outgoing_he.end_vertex)
                    .or_else(error_none!("End vertex not found"))?
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

        Some(())
    }
}

#[cfg(test)]
mod tests {
    use crate::ops::insert::InsertOrGetEdge;

    use super::*;
    use glam::Vec3;

    #[test]
    fn test_remove_degenerate_faces() {
        crate::utils::get_tracing_subscriber();
        let mut meshgraph = MeshGraph::new();

        let center_v_id = meshgraph.insert_vertex(Vec3::new(0.0, 0.0, 1.0));

        let v1_id = meshgraph.insert_vertex(Vec3::new(-0.2, 0.0, 0.0));
        let he_c_1_id = meshgraph
            .insert_or_get_edge(center_v_id, v1_id)
            .start_to_end_he_id;
        let v1p_id = meshgraph.insert_vertex(Vec3::new(-0.2, 0.0, 0.0));
        let InsertOrGetEdge {
            start_to_end_he_id: he_c_1p_id,
            twin_he_id: he_1p_c_id,
            ..
        } = meshgraph.insert_or_get_edge(center_v_id, v1p_id);

        let v2_id = meshgraph.insert_vertex(Vec3::new(-1.0, 1.0, 0.0));
        let he_c_2_id = meshgraph
            .insert_or_get_edge(center_v_id, v2_id)
            .start_to_end_he_id;

        let v3_id = meshgraph.insert_vertex(Vec3::new(-1.0, -1.0, 0.0));
        let he_c_3_id = meshgraph
            .insert_or_get_edge(center_v_id, v3_id)
            .start_to_end_he_id;

        let v4_id = meshgraph.insert_vertex(Vec3::new(0.2, 0.0, 0.0));
        let he_c_4_id = meshgraph
            .insert_or_get_edge(center_v_id, v4_id)
            .start_to_end_he_id;

        let v4p_id = meshgraph.insert_vertex(Vec3::new(0.2, 0.0, 0.0));
        let InsertOrGetEdge {
            start_to_end_he_id: he_c_4p_id,
            twin_he_id: he_4p_c_id,
            ..
        } = meshgraph.insert_or_get_edge(center_v_id, v4p_id);

        let v5_id = meshgraph.insert_vertex(Vec3::new(1.0, -1.0, 0.0));
        let he_c_5_id = meshgraph
            .insert_or_get_edge(center_v_id, v5_id)
            .start_to_end_he_id;

        let v6_id = meshgraph.insert_vertex(Vec3::new(1.0, 1.0, 0.0));
        let he_c_6_id = meshgraph
            .insert_or_get_edge(center_v_id, v6_id)
            .start_to_end_he_id;

        meshgraph
            .create_face_from_halfedges(he_c_1_id, he_c_2_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_2_id, he_c_3_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_3_id, he_c_1p_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(he_c_1p_id, he_c_4p_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(he_c_4p_id, he_c_5_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_5_id, he_c_6_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_6_id, he_c_4_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(he_c_1_id, he_c_4_id)
            .unwrap();

        meshgraph.halfedges[he_c_1p_id].end_vertex = v1_id;
        meshgraph.halfedges[he_c_4p_id].end_vertex = v4_id;

        // already created from face above
        let InsertOrGetEdge {
            start_to_end_he_id: he_1p_4p_id,
            twin_he_id: he_4p_1p_id,
            ..
        } = meshgraph.insert_or_get_edge(v1p_id, v4p_id);
        meshgraph.halfedges[he_1p_4p_id].end_vertex = v4_id;
        meshgraph.halfedges[he_4p_1p_id].end_vertex = v1_id;

        let InsertOrGetEdge {
            start_to_end_he_id: he_3_1p_id,
            twin_he_id: he_1p_3_id,
            ..
        } = meshgraph.insert_or_get_edge(v3_id, v1p_id);
        meshgraph.halfedges[he_3_1p_id].end_vertex = v1_id;

        let InsertOrGetEdge {
            start_to_end_he_id: he_4p_5_id,
            twin_he_id: he_5_4p_id,
            ..
        } = meshgraph.insert_or_get_edge(v4p_id, v5_id);
        meshgraph.halfedges[he_5_4p_id].end_vertex = v4_id;

        meshgraph.outgoing_halfedges[v1_id].push(he_1p_4p_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_c_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_3_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_1p_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_c_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_5_id);

        meshgraph.delete_only_vertex(v1p_id);
        meshgraph.delete_only_vertex(v4p_id);

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let mut removed_vertices = vec![];
        let mut removed_halfedges = vec![];
        let mut removed_faces = vec![];

        meshgraph.remove_degenerate_faces(
            center_v_id,
            &mut removed_vertices,
            &mut removed_halfedges,
            &mut removed_faces,
        );

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }
    }

    #[test]
    fn test_remove_degenerate_edges() {
        crate::utils::get_tracing_subscriber();

        let mut meshgraph = MeshGraph::new();

        let center_v_id = meshgraph.insert_vertex(Vec3::new(0.0, 0.0, 1.0));

        let v1_id = meshgraph.insert_vertex(Vec3::new(0.0, 0.0, 0.0));
        let he_c_1_id = meshgraph
            .insert_or_get_edge(center_v_id, v1_id)
            .start_to_end_he_id;

        let v1p_id = meshgraph.insert_vertex(Vec3::new(0.0, 0.0, 0.0));
        let InsertOrGetEdge {
            start_to_end_he_id: he_c_1p_id,
            twin_he_id: he_1p_c_id,
            ..
        } = meshgraph.insert_or_get_edge(center_v_id, v1p_id);

        let v2_id = meshgraph.insert_vertex(Vec3::new(-1.0, 1.0, 0.0));
        let he_c_2_id = meshgraph
            .insert_or_get_edge(center_v_id, v2_id)
            .start_to_end_he_id;

        let v3_id = meshgraph.insert_vertex(Vec3::new(-1.0, -1.0, 0.0));
        let he_c_3_id = meshgraph
            .insert_or_get_edge(center_v_id, v3_id)
            .start_to_end_he_id;

        let v5_id = meshgraph.insert_vertex(Vec3::new(1.0, -1.0, 0.0));
        let he_c_5_id = meshgraph
            .insert_or_get_edge(center_v_id, v5_id)
            .start_to_end_he_id;

        let v6_id = meshgraph.insert_vertex(Vec3::new(1.0, 1.0, 0.0));
        let he_c_6_id = meshgraph
            .insert_or_get_edge(center_v_id, v6_id)
            .start_to_end_he_id;

        meshgraph
            .create_face_from_halfedges(he_c_1_id, he_c_2_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_2_id, he_c_3_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_3_id, he_c_1p_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(he_c_1p_id, he_c_5_id)
            .unwrap();

        meshgraph
            .create_face_from_halfedges(he_c_5_id, he_c_6_id)
            .unwrap();
        meshgraph
            .create_face_from_halfedges(he_c_6_id, he_c_1_id)
            .unwrap();

        meshgraph.halfedges[he_c_1p_id].end_vertex = v1_id;

        // already created from face above
        let InsertOrGetEdge {
            start_to_end_he_id: he_3_1p_id,
            twin_he_id: he_1p_3_id,
            ..
        } = meshgraph.insert_or_get_edge(v3_id, v1p_id);
        meshgraph.halfedges[he_3_1p_id].end_vertex = v1_id;

        let InsertOrGetEdge {
            start_to_end_he_id: he_1p_5_id,
            twin_he_id: he_5_1p_id,
            ..
        } = meshgraph.insert_or_get_edge(v1p_id, v5_id);
        meshgraph.halfedges[he_5_1p_id].end_vertex = v1_id;

        meshgraph.outgoing_halfedges[v1_id].push(he_1p_c_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_5_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_3_id);

        meshgraph.delete_only_vertex(v1p_id);

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        meshgraph.remove_degenerate_edges(center_v_id);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }
    }
}
