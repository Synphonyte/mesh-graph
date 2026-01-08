use hashbrown::HashSet;
use itertools::Itertools;
use tracing::{error, info, instrument};

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

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
    ) -> (Vec<VertexId>, Vec<HalfedgeId>, Vec<FaceId>) {
        let mut removed_vertices = Vec::new();
        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        let mut vertices = vec![vertex_id];

        loop {
            let (
                step_inserted_duplicated_vertex,
                step_removed_vertices,
                step_removed_halfedges,
                step_removed_faces,
            ) = self.make_vertex_neighborhood_manifold_step(vertices[0]);

            if let Some(inserted_vertex) = step_inserted_duplicated_vertex {
                vertices.push(inserted_vertex);
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

            removed_vertices.extend(step_removed_vertices);
            removed_halfedges.extend(step_removed_halfedges);
            removed_faces.extend(step_removed_faces);
        }

        (removed_vertices, removed_halfedges, removed_faces)
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
        if self.vertices.get(vertex_id).is_none() {
            return None;
        }

        self.remove_neighboring_flaps(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        );

        if let Some(inserted_duplicated_vertex) = self.remove_degenerate_faces(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        ) {
            return Some(inserted_duplicated_vertex);
        }

        if let Some(inserted_duplicated_vertex) = self.remove_degenerate_edges(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        ) {
            return Some(inserted_duplicated_vertex);
        }

        None
    }

    fn remove_degenerate_edges(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<VertexId> {
        todo!()
    }

    fn remove_degenerate_faces(
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
                let outgoing_halfedge_ids = self.vertices[vertex_id]
                    .outgoing_halfedges(self)
                    .collect_vec();
                let mut start_idx = 0;

                for (idx, he_id) in outgoing_halfedge_ids.iter().enumerate() {
                    let he = self
                        .halfedges
                        .get(*he_id)
                        .or_else(error_none!("Halfedge not found"))?;

                    if he.face == Some(face_id1) {
                        start_idx = idx;
                        break;
                    }
                }

                let mut end_idx = start_idx;
                let mut side_one = vec![];

                loop {
                    let he_id = outgoing_halfedge_ids[end_idx];

                    let he = self
                        .halfedges
                        .get(he_id)
                        .or_else(error_none!("Halfedge not found"))?;

                    if he.face == Some(face_id2) {
                        break;
                    }

                    side_one.push(he_id);

                    end_idx += 1;
                    end_idx %= outgoing_halfedge_ids.len();
                }

                let mut side_two = vec![];

                while end_idx != start_idx {
                    let he_id = outgoing_halfedge_ids[end_idx];

                    side_two.push(he_id);

                    end_idx += 1;
                    end_idx %= outgoing_halfedge_ids.len();
                }

                info!("before");

                let new_vertex_id =
                    self.split_regions_at_vertex(vertex_id, side_one, side_two, removed_halfedges);

                info!("after");

                let (del_v_ids, del_he_ids) = self.delete_face(face_id1);
                removed_vertices.extend(del_v_ids);
                removed_halfedges.extend(del_he_ids);
                removed_faces.push(face_id1);

                let (del_v_ids, del_he_ids) = self.delete_face(face_id2);
                removed_vertices.extend(del_v_ids);
                removed_halfedges.extend(del_he_ids);
                removed_faces.push(face_id2);

                return new_vertex_id;
            }
        }

        None
    }

    #[instrument(skip(self))]
    fn split_regions_at_vertex(
        &mut self,
        vertex_id: VertexId,
        side_one: Vec<HalfedgeId>,
        side_two: Vec<HalfedgeId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
    ) -> Option<VertexId> {
        if side_one.len() < 2 || side_two.len() < 2 {
            error!("Not enough halfedges to split");
            return None;
        }

        #[cfg(feature = "rerun")]
        {
            self.log_vert_rerun("split_regions_at_vertex", vertex_id);
            self.log_hes_rerun("split_regions_at_vertex/side_one", &side_one);
            self.log_hes_rerun("split_regions_at_vertex/side_two", &side_two);
        }

        let first_he_id = side_one[0];
        let last_he_id = side_one[side_one.len() - 1];

        self.weld_halfedges(vertex_id, last_he_id, first_he_id, removed_halfedges);

        let new_vertex_id = self.insert_vertex(
            *self
                .positions
                .get(vertex_id)
                .or_else(error_none!("Vertex position not found"))?,
        );

        // TODO : Move vertices apart

        // Updating start vertex
        for he_id in &side_two {
            let he = self
                .halfedges
                .get(*he_id)
                .or_else(error_none!("Halfedge not found"))?;

            self.halfedges
                .get_mut(he.twin.or_else(error_none!("Twin not found"))?)
                .or_else(error_none!("Twin not found"))?
                .end_vertex = new_vertex_id;

            self.outgoing_halfedges[new_vertex_id].push(*he_id);
        }

        // we just inserted the new vertex
        self.vertices[new_vertex_id].outgoing_halfedge = Some(side_two[0]);

        self.weld_halfedges(
            new_vertex_id,
            side_two[0],
            side_two[side_two.len() - 1],
            removed_halfedges,
        );

        Some(new_vertex_id)
    }

    /// The two halfedges are actually coincident. `he_id2` is removed and replaced by `he_id1`.
    /// `start_vertex_id` is the shared start vertex of both halfedges.
    fn weld_halfedges(
        &mut self,
        start_vertex_id: VertexId,
        he_id1: HalfedgeId,
        he_id2: HalfedgeId,
        removed_halfedges: &mut Vec<HalfedgeId>,
    ) -> Option<()> {
        let he2 = self
            .halfedges
            .get(he_id2)
            .or_else(error_none!("Halfedge not found"))?;

        let he_twin2_id = he2.twin.or_else(error_none!("Twin halfedge not found"))?;

        let prev_he_id = he2
            .prev(self)
            .or_else(error_none!("Prev halfedge id not found"))?;

        let he_twin1_id = self
            .halfedges
            .get(he_id1)
            .or_else(error_none!("Halfedge not found"))?
            .twin
            .or_else(error_none!("Twin halfedge not found"))?;

        self.delete_only_halfedge(he_id2);
        self.delete_only_halfedge(he_twin1_id);
        removed_halfedges.push(he_id2);
        removed_halfedges.push(he_twin1_id);

        self.halfedges
            .get_mut(prev_he_id)
            .or_else(error_none!("Prev halfedge not found"))?
            .next = Some(he_id1);

        self.halfedges
            .get_mut(he_twin2_id)
            .or_else(error_none!("Twin halfedge not found"))?
            .twin = Some(he_id1);

        self.halfedges
            .get_mut(he_id1)
            .or_else(error_none!("Halfedge not found"))?
            .twin = Some(he_twin2_id);

        self.vertices
            .get_mut(start_vertex_id)
            .or_else(error_none!("Vertex not found"))?
            .outgoing_halfedge = Some(he_id1);

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

        let mut face_tuples = faces.into_iter().circular_tuple_windows().collect_vec();

        let mut first = true;

        while let Some((face_id1, face_id2)) = face_tuples.pop() {
            #[cfg(feature = "rerun")]
            {
                self.log_rerun();
            }

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
    use glam::Vec3;
    use tracing_subscriber::EnvFilter;

    use super::*;

    #[test]
    fn test_remove_degenerate_faces() {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .with_line_number(true)
            .init();

        let mut meshgraph = MeshGraph::new();

        let center_v_id = meshgraph.insert_vertex(Vec3::new(0.0, 0.0, 1.0));

        let v1_id = meshgraph.insert_vertex(Vec3::new(-0.2, 0.0, 0.0));
        let (he_c_1_id, _) = meshgraph.insert_or_get_edge(center_v_id, v1_id);

        let v1p_id = meshgraph.insert_vertex(Vec3::new(-0.2, 0.0, 0.0));
        let (he_c_1p_id, he_1p_c_id) = meshgraph.insert_or_get_edge(center_v_id, v1p_id);

        let v2_id = meshgraph.insert_vertex(Vec3::new(-1.0, 1.0, 0.0));
        let (he_c_2_id, _) = meshgraph.insert_or_get_edge(center_v_id, v2_id);

        let v3_id = meshgraph.insert_vertex(Vec3::new(-1.0, -1.0, 0.0));
        let (he_c_3_id, _) = meshgraph.insert_or_get_edge(center_v_id, v3_id);

        let v4_id = meshgraph.insert_vertex(Vec3::new(0.2, 0.0, 0.0));
        let (he_c_4_id, _) = meshgraph.insert_or_get_edge(center_v_id, v4_id);

        let v4p_id = meshgraph.insert_vertex(Vec3::new(0.2, 0.0, 0.0));
        let (he_c_4p_id, he_4p_c_id) = meshgraph.insert_or_get_edge(center_v_id, v4p_id);

        let v5_id = meshgraph.insert_vertex(Vec3::new(1.0, -1.0, 0.0));
        let (he_c_5_id, _) = meshgraph.insert_or_get_edge(center_v_id, v5_id);

        let v6_id = meshgraph.insert_vertex(Vec3::new(1.0, 1.0, 0.0));
        let (he_c_6_id, _) = meshgraph.insert_or_get_edge(center_v_id, v6_id);

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
        let (he_1p_4p_id, he_4p_1p_id) = meshgraph.insert_or_get_edge(v1p_id, v4p_id);
        meshgraph.halfedges[he_1p_4p_id].end_vertex = v4_id;
        meshgraph.halfedges[he_4p_1p_id].end_vertex = v1_id;

        let (he_3_1p_id, he_1p_3_id) = meshgraph.insert_or_get_edge(v3_id, v1p_id);
        meshgraph.halfedges[he_3_1p_id].end_vertex = v1_id;

        let (he_4p_5_id, he_5_4p_id) = meshgraph.insert_or_get_edge(v4p_id, v5_id);
        meshgraph.halfedges[he_5_4p_id].end_vertex = v4_id;

        meshgraph.outgoing_halfedges[v1_id].push(he_1p_4p_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_c_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_3_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_1p_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_c_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_5_id);

        meshgraph.delete_only_vertex(v1p_id);
        meshgraph.delete_only_vertex(v4p_id);

        // #[cfg(feature = "rerun")]
        // meshgraph.log_rerun();

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
            info!("finished");
            meshgraph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }
    }

    #[test]
    fn test_remove_degenerate_edges() {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .with_line_number(true)
            .init();

        // TODO : create test mesh
    }
}
