use hashbrown::HashSet;
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

#[derive(Default)]
pub struct VertexNeighborhoodCleanup {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    pub added_vertices: Vec<VertexId>,
}

#[derive(Default)]
struct VertexNeighborhoodCleanupStep {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    pub added_duplicated_vertices: Vec<VertexId>,
    pub touched_vertices: Vec<VertexId>,
}

impl MeshGraph {
    /// Ensure that the neighborhood of a vertex is manifold.
    ///
    /// It removes flaps (two neighboring coincident triangles) and splits the given vertex
    /// in two if there are non-neighboring degenerate triangles or edges.
    ///
    /// See [Freestyle: Sculpting meshes with self-adaptive topology DOI 10.1016/j.cag.2011.03.033](https://inria.hal.science/inria-00606516v1/document)
    /// Chapters 3.2 and 5.1
    #[instrument(skip(self))]
    pub fn make_vertex_neighborhood_manifold(
        &mut self,
        vertex_id: VertexId,
    ) -> VertexNeighborhoodCleanup {
        #[cfg(feature = "rerun")]
        self.log_vert_rerun("make_neigh_manifold", vertex_id);

        let mut result = VertexNeighborhoodCleanup::default();

        let mut vertices = vec![vertex_id];

        while let Some(&v_id) = vertices.first() {
            if !self.vertices.contains_key(v_id) {
                vertices.swap_remove(0);
                continue;
            }

            let VertexNeighborhoodCleanupStep {
                added_duplicated_vertices,
                touched_vertices,
                removed_vertices,
                removed_halfedges,
                removed_faces,
            } = self.make_vertex_neighborhood_manifold_step(v_id);

            if removed_vertices.is_empty()
                && removed_halfedges.is_empty()
                && removed_faces.is_empty()
                && added_duplicated_vertices.is_empty()
                && touched_vertices.is_empty()
            {
                vertices.swap_remove(0);
            }

            vertices.extend(touched_vertices);
            for added_vertex in added_duplicated_vertices {
                vertices.push(added_vertex);

                result.added_vertices.push(added_vertex);
            }

            result.removed_vertices.extend(removed_vertices);
            result.removed_halfedges.extend(removed_halfedges);
            result.removed_faces.extend(removed_faces);

            if vertices.is_empty() {
                break;
            }
        }

        for &cancelled_v_id in
            HashSet::<VertexId>::from_iter(result.removed_vertices.iter().copied())
                .intersection(&HashSet::from_iter(result.added_vertices.iter().copied()))
        {
            result.added_vertices.retain(|&v_id| v_id != cancelled_v_id);
            result
                .removed_vertices
                .retain(|&v_id| v_id != cancelled_v_id);
        }

        result
    }

    fn make_vertex_neighborhood_manifold_step(
        &mut self,
        vertex_id: VertexId,
    ) -> VertexNeighborhoodCleanupStep {
        let mut removed_vertices = Vec::new();
        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        let added_duplicated_vertices = self
            .make_vertex_neighborhood_manifold_inner(
                vertex_id,
                &mut removed_vertices,
                &mut removed_halfedges,
                &mut removed_faces,
            )
            .unwrap_or_default();

        VertexNeighborhoodCleanupStep {
            added_duplicated_vertices,
            touched_vertices: vec![],
            removed_vertices,
            removed_halfedges,
            removed_faces,
        }
    }

    pub fn make_vertex_neighborhood_manifold_inner(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<Vec<VertexId>> {
        self.vertices.get(vertex_id)?;

        if self.remove_single_face(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        )? {
            return Some(vec![]);
        }

        if self.remove_neighboring_flaps(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        )? {
            return Some(vec![]);
        }

        if let Some(new_vertices) = self.split_disconnected_neighborhoods(vertex_id)
            && !new_vertices.is_empty()
        {
            return Some(new_vertices);
        }

        if let Some(inserted_duplicated_vertex) = self.remove_degenerate_faces(
            vertex_id,
            removed_vertices,
            removed_halfedges,
            removed_faces,
        ) {
            return Some(vec![inserted_duplicated_vertex]);
        }

        self.remove_degenerate_edges(vertex_id).map(|v| vec![v])
    }

    fn split_disconnected_neighborhoods(&mut self, vertex_id: VertexId) -> Option<Vec<VertexId>> {
        let mut new_vertices = Vec::new();

        let mut outgoing_halfedges = HashSet::<HalfedgeId>::from_iter(
            self.outgoing_halfedges
                .get(vertex_id)
                .or_else(error_none!("Outgoing halfedges not found"))?
                .iter()
                .copied(),
        );

        while let Some(&start_he_id) = outgoing_halfedges.iter().next() {
            let mut current_he_id = start_he_id;

            let mut current_outgoing_halfedges = Vec::with_capacity(outgoing_halfedges.len());

            loop {
                outgoing_halfedges.remove(&current_he_id);
                current_outgoing_halfedges.push(current_he_id);

                let cur_he = self
                    .halfedges
                    .get(current_he_id)
                    .or_else(error_none!("Halfedge not found"))?;

                let Some(next_he_id) = cur_he.cw_rotated_neighbour(self) else {
                    // TODO : handle boundary edges?
                    break;
                };

                if next_he_id == start_he_id {
                    break;
                }

                current_he_id = next_he_id;
            }

            if !outgoing_halfedges.is_empty() {
                let new_vertex_id = self
                    .duplicate_vertex_and_assign_halfedges(vertex_id, current_outgoing_halfedges)?;

                new_vertices.push(new_vertex_id);

                self.vertices
                    .get_mut(vertex_id)
                    .or_else(error_none!("Vertex not found"))?
                    .outgoing_halfedge = Some(*outgoing_halfedges.iter().next().unwrap());

                self.outgoing_halfedges
                    .insert(vertex_id, Vec::from_iter(outgoing_halfedges));

                return Some(new_vertices);
            }
        }

        Some(new_vertices)
    }

    fn duplicate_vertex_and_assign_halfedges(
        &mut self,
        vertex_id: VertexId,
        outgoing_halfedges: Vec<HalfedgeId>,
    ) -> Option<VertexId> {
        let new_vert_id = self.add_vertex(
            *self
                .positions
                .get(vertex_id)
                .or_else(error_none!("Position not found"))?
                + glam::Vec3::new(0.1, 0.0, 0.0),
        );

        tracing::info!("Duplicated {vertex_id:?}: {new_vert_id:?}");

        for &he_id in &outgoing_halfedges {
            let he = self
                .halfedges
                .get(he_id)
                .or_else(error_none!("Halfedge not found"))?;

            let twin_id = he.twin.or_else(error_none!("Twin not found"))?;

            self.halfedges
                .get_mut(twin_id)
                .or_else(error_none!("Twin not found"))?
                .end_vertex = new_vert_id;
        }

        // just created above
        self.vertices[new_vert_id].outgoing_halfedge = Some(outgoing_halfedges[0]);

        self.outgoing_halfedges
            .insert(new_vert_id, outgoing_halfedges);

        Some(new_vert_id)
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

        let new_vertex_id = self.add_vertex(
            *self
                .positions
                .get(vertex_id)
                .or_else(error_none!("Vertex position not found"))?,
        );

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

    #[instrument(skip_all)]
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

                let (del_v_ids, del_he_ids) = self.remove_face(face_id1);
                removed_vertices.extend(del_v_ids);
                removed_halfedges.extend(del_he_ids);
                removed_faces.push(face_id1);

                let (del_v_ids, del_he_ids) = self.remove_face(face_id2);
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

        let new_vertex_id = self.add_vertex(
            *self
                .positions
                .get(vertex_id)
                .or_else(error_none!("Vertex position not found"))?,
        );

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

    pub(crate) fn remove_single_face(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<bool> {
        let face_ids = self
            .outgoing_halfedges
            .get(vertex_id)
            .or_else(error_none!(
                "Outgoing halfedges not found for vertex {vertex_id:?}"
            ))?
            .iter()
            .filter_map(|&he_id| {
                self.halfedges
                    .get(he_id)
                    .or_else(error_none!("Halfedge not found"))
                    .and_then(|he| he.face)
            })
            .collect_vec();

        if face_ids.len() == 1 {
            let face_id = face_ids[0];

            let (v_ids, he_ids) = self.remove_face(face_id);

            removed_vertices.extend(v_ids);
            removed_halfedges.extend(he_ids);
            removed_faces.push(face_id);

            Some(true)
        } else {
            Some(false)
        }
    }

    fn remove_neighboring_flaps(
        &mut self,
        vertex_id: VertexId,
        removed_vertices: &mut Vec<VertexId>,
        removed_halfedges: &mut Vec<HalfedgeId>,
        removed_faces: &mut Vec<FaceId>,
    ) -> Option<bool> {
        let faces = self
            .vertices
            .get(vertex_id)
            .or_else(error_none!("Vertex not found"))?
            .faces(self)
            .collect_vec();

        if faces.len() < 2 {
            return Some(false);
        }

        let mut face_tuples = faces.into_iter().circular_tuple_windows().collect_vec();

        while let Some((face_id1, face_id2)) = face_tuples.pop() {
            if self.faces_share_all_vertices(face_id1, face_id2) {
                #[cfg(feature = "rerun")]
                {
                    self.log_vert_rerun("flap", vertex_id);
                    self.log_face_rerun("flap1", face_id1);
                    self.log_face_rerun("flap2", face_id2);
                }

                let mut halfedges_of_faces = HashSet::<HalfedgeId>::from_iter(
                    self.halfedges.iter().filter_map(|(he_id, he)| {
                        if he.face == Some(face_id1) || he.face == Some(face_id2) {
                            Some(he_id)
                        } else {
                            None
                        }
                    }),
                );

                let (vs, hes) = self.remove_face(face_id1);
                for he_id in &hes {
                    halfedges_of_faces.remove(he_id);
                }
                removed_vertices.extend(vs);
                removed_halfedges.extend(hes);
                removed_faces.push(face_id1);

                let (vs, hes) = self.remove_face(face_id2);
                for he_id in &hes {
                    halfedges_of_faces.remove(he_id);
                }
                removed_vertices.extend(vs);
                removed_halfedges.extend(hes);
                removed_faces.push(face_id2);

                // TODO : Handle the case when there is only one halfedge?
                if halfedges_of_faces.len() >= 2 {
                    for (he_id1, he_id2) in halfedges_of_faces.iter().copied().tuple_combinations()
                    {
                        let Some(he1) = self.halfedges.get(he_id1) else {
                            // might have been deleted by prior iteration
                            continue;
                        };
                        let twin_id1 = he1.twin.or_else(error_none!("Twin 1 missing"))?;

                        let Some(he2) = self.halfedges.get(he_id2) else {
                            continue;
                        };
                        let twin_id2 = he2.twin.or_else(error_none!("Twin 2 missing"))?;

                        let start_v_id1 = he1
                            .start_vertex(self)
                            .or_else(error_none!("Start vertex 1 missing"))?;
                        let start_v_id2 = he2
                            .start_vertex(self)
                            .or_else(error_none!("Start vertex 2 missing"))?;

                        if he1.end_vertex != start_v_id2 || he2.end_vertex != start_v_id1 {
                            continue;
                        }

                        // the twin edges of the neighboring faces of the deleted faces are still there
                        // we need to remove them and re-connect (twin) their twin edges

                        self.remove_only_halfedge(he_id1);
                        self.remove_only_halfedge(he_id2);
                        removed_halfedges.push(he_id1);
                        removed_halfedges.push(he_id2);

                        {
                            let twin1 = self
                                .halfedges
                                .get_mut(twin_id1)
                                .or_else(error_none!("Twin 1 missing"))?;
                            twin1.twin = Some(twin_id2);
                        };

                        {
                            let twin2 = self
                                .halfedges
                                .get_mut(twin_id2)
                                .or_else(error_none!("Twin 2 missing"))?;
                            twin2.twin = Some(twin_id1);
                        };

                        self.vertices
                            .get_mut(start_v_id1)
                            .or_else(error_none!("Start vertex 1 missing"))?
                            .outgoing_halfedge = Some(twin_id2);

                        self.vertices
                            .get_mut(start_v_id2)
                            .or_else(error_none!("Start vertex 2 missing"))?
                            .outgoing_halfedge = Some(twin_id1);
                    }
                } else {
                    tracing::warn!("There's only one halfedge left");
                }

                return Some(true);
            }
        }

        Some(false)
    }
}

#[cfg(test)]
mod tests {
    use crate::ops::AddOrGetEdge;

    use super::*;
    use glam::Vec3;

    #[test]
    fn test_remove_degenerate_faces() {
        crate::utils::get_tracing_subscriber();
        let mut meshgraph = MeshGraph::new();

        let center_v_id = meshgraph.add_vertex(Vec3::new(0.0, 0.0, 1.0));

        let v1_id = meshgraph.add_vertex(Vec3::new(-0.2, 0.0, 0.0));
        let he_c_1_id = meshgraph
            .add_or_get_edge(center_v_id, v1_id)
            .start_to_end_he_id;
        let v1p_id = meshgraph.add_vertex(Vec3::new(-0.2, 0.0, 0.0));
        let AddOrGetEdge {
            start_to_end_he_id: he_c_1p_id,
            twin_he_id: he_1p_c_id,
            ..
        } = meshgraph.add_or_get_edge(center_v_id, v1p_id);

        let v2_id = meshgraph.add_vertex(Vec3::new(-1.0, 1.0, 0.0));
        let he_c_2_id = meshgraph
            .add_or_get_edge(center_v_id, v2_id)
            .start_to_end_he_id;

        let v3_id = meshgraph.add_vertex(Vec3::new(-1.0, -1.0, 0.0));
        let he_c_3_id = meshgraph
            .add_or_get_edge(center_v_id, v3_id)
            .start_to_end_he_id;

        let v4_id = meshgraph.add_vertex(Vec3::new(0.2, 0.0, 0.0));
        let he_c_4_id = meshgraph
            .add_or_get_edge(center_v_id, v4_id)
            .start_to_end_he_id;

        let v4p_id = meshgraph.add_vertex(Vec3::new(0.2, 0.0, 0.0));
        let AddOrGetEdge {
            start_to_end_he_id: he_c_4p_id,
            twin_he_id: he_4p_c_id,
            ..
        } = meshgraph.add_or_get_edge(center_v_id, v4p_id);

        let v5_id = meshgraph.add_vertex(Vec3::new(1.0, -1.0, 0.0));
        let he_c_5_id = meshgraph
            .add_or_get_edge(center_v_id, v5_id)
            .start_to_end_he_id;

        let v6_id = meshgraph.add_vertex(Vec3::new(1.0, 1.0, 0.0));
        let he_c_6_id = meshgraph
            .add_or_get_edge(center_v_id, v6_id)
            .start_to_end_he_id;

        meshgraph
            .add_face_from_halfedges(he_c_1_id, he_c_2_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_2_id, he_c_3_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_3_id, he_c_1p_id)
            .unwrap();

        meshgraph
            .add_face_from_halfedges(he_c_1p_id, he_c_4p_id)
            .unwrap();

        meshgraph
            .add_face_from_halfedges(he_c_4p_id, he_c_5_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_5_id, he_c_6_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_6_id, he_c_4_id)
            .unwrap();

        meshgraph
            .add_face_from_halfedges(he_c_1_id, he_c_4_id)
            .unwrap();

        meshgraph.halfedges[he_c_1p_id].end_vertex = v1_id;
        meshgraph.halfedges[he_c_4p_id].end_vertex = v4_id;

        // already created from face above
        let AddOrGetEdge {
            start_to_end_he_id: he_1p_4p_id,
            twin_he_id: he_4p_1p_id,
            ..
        } = meshgraph.add_or_get_edge(v1p_id, v4p_id);
        meshgraph.halfedges[he_1p_4p_id].end_vertex = v4_id;
        meshgraph.halfedges[he_4p_1p_id].end_vertex = v1_id;

        let AddOrGetEdge {
            start_to_end_he_id: he_3_1p_id,
            twin_he_id: he_1p_3_id,
            ..
        } = meshgraph.add_or_get_edge(v3_id, v1p_id);
        meshgraph.halfedges[he_3_1p_id].end_vertex = v1_id;

        let AddOrGetEdge {
            start_to_end_he_id: he_4p_5_id,
            twin_he_id: he_5_4p_id,
            ..
        } = meshgraph.add_or_get_edge(v4p_id, v5_id);
        meshgraph.halfedges[he_5_4p_id].end_vertex = v4_id;

        meshgraph.outgoing_halfedges[v1_id].push(he_1p_4p_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_c_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_3_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_1p_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_c_id);
        meshgraph.outgoing_halfedges[v4_id].push(he_4p_5_id);

        meshgraph.remove_only_vertex(v1p_id);
        meshgraph.remove_only_vertex(v4p_id);

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

        let center_v_id = meshgraph.add_vertex(Vec3::new(0.0, 0.0, 1.0));

        let v1_id = meshgraph.add_vertex(Vec3::new(0.0, 0.0, 0.0));
        let he_c_1_id = meshgraph
            .add_or_get_edge(center_v_id, v1_id)
            .start_to_end_he_id;

        let v1p_id = meshgraph.add_vertex(Vec3::new(0.0, 0.0, 0.0));
        let AddOrGetEdge {
            start_to_end_he_id: he_c_1p_id,
            twin_he_id: he_1p_c_id,
            ..
        } = meshgraph.add_or_get_edge(center_v_id, v1p_id);

        let v2_id = meshgraph.add_vertex(Vec3::new(-1.0, 1.0, 0.0));
        let he_c_2_id = meshgraph
            .add_or_get_edge(center_v_id, v2_id)
            .start_to_end_he_id;

        let v3_id = meshgraph.add_vertex(Vec3::new(-1.0, -1.0, 0.0));
        let he_c_3_id = meshgraph
            .add_or_get_edge(center_v_id, v3_id)
            .start_to_end_he_id;

        let v5_id = meshgraph.add_vertex(Vec3::new(1.0, -1.0, 0.0));
        let he_c_5_id = meshgraph
            .add_or_get_edge(center_v_id, v5_id)
            .start_to_end_he_id;

        let v6_id = meshgraph.add_vertex(Vec3::new(1.0, 1.0, 0.0));
        let he_c_6_id = meshgraph
            .add_or_get_edge(center_v_id, v6_id)
            .start_to_end_he_id;

        meshgraph
            .add_face_from_halfedges(he_c_1_id, he_c_2_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_2_id, he_c_3_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_3_id, he_c_1p_id)
            .unwrap();

        meshgraph
            .add_face_from_halfedges(he_c_1p_id, he_c_5_id)
            .unwrap();

        meshgraph
            .add_face_from_halfedges(he_c_5_id, he_c_6_id)
            .unwrap();
        meshgraph
            .add_face_from_halfedges(he_c_6_id, he_c_1_id)
            .unwrap();

        meshgraph.halfedges[he_c_1p_id].end_vertex = v1_id;

        // already created from face above
        let AddOrGetEdge {
            start_to_end_he_id: he_3_1p_id,
            twin_he_id: he_1p_3_id,
            ..
        } = meshgraph.add_or_get_edge(v3_id, v1p_id);
        meshgraph.halfedges[he_3_1p_id].end_vertex = v1_id;

        let AddOrGetEdge {
            start_to_end_he_id: he_1p_5_id,
            twin_he_id: he_5_1p_id,
            ..
        } = meshgraph.add_or_get_edge(v1p_id, v5_id);
        meshgraph.halfedges[he_5_1p_id].end_vertex = v1_id;

        meshgraph.outgoing_halfedges[v1_id].push(he_1p_c_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_5_id);
        meshgraph.outgoing_halfedges[v1_id].push(he_1p_3_id);

        meshgraph.remove_only_vertex(v1p_id);

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        meshgraph.remove_degenerate_edges(center_v_id);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }
    }

    #[test]
    fn test_remove_flap() {
        crate::utils::get_tracing_subscriber();

        let mut mesh_graph = MeshGraph::new();

        let v1 = mesh_graph.add_vertex(Vec3::new(0.0, 0.0, 0.0));
        let v2 = mesh_graph.add_vertex(Vec3::new(1.0, 0.0, 0.0));
        let v3 = mesh_graph.add_vertex(Vec3::new(0.0, 1.0, 0.0));

        let v4 = mesh_graph.add_vertex(Vec3::new(1.0, 1.0, 0.5));
        let v5 = mesh_graph.add_vertex(Vec3::new(1.0, 1.0, -0.5));

        let edge1 = mesh_graph.add_edge(v1, v2);
        let edge2 = mesh_graph.add_edge(v2, v3);
        let edge2_d = mesh_graph.add_edge(v2, v3);

        mesh_graph.add_face_from_halfedges(edge1.start_to_end_he_id, edge2.start_to_end_he_id);
        mesh_graph.add_face_from_halfedges(edge2_d.twin_he_id, edge1.twin_he_id);

        mesh_graph.add_face_from_halfedge_and_vertex(edge2.twin_he_id, v4);
        mesh_graph.add_face_from_halfedge_and_vertex(edge2_d.start_to_end_he_id, v5);

        #[cfg(feature = "rerun")]
        mesh_graph.log_rerun();

        let mut removed_vertices = Vec::new();
        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        mesh_graph.remove_neighboring_flaps(
            v1,
            &mut removed_vertices,
            &mut removed_halfedges,
            &mut removed_faces,
        );

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        assert_eq!(removed_vertices.len(), 1);
        assert_eq!(removed_halfedges.len(), 6);
        assert_eq!(removed_faces.len(), 2);
    }

    #[test]
    fn test_remove_double_flaps() {
        crate::utils::get_tracing_subscriber();

        let mut mesh_graph = MeshGraph::new();

        let v1 = mesh_graph.add_vertex(Vec3::new(0.0, 0.0, 0.0));
        let v2 = mesh_graph.add_vertex(Vec3::new(1.0, 0.0, 0.0));
        let v3 = mesh_graph.add_vertex(Vec3::new(0.0, 1.0, 0.0));
        let v4 = mesh_graph.add_vertex(Vec3::new(1.0, 1.0, 0.0));

        let v5 = mesh_graph.add_vertex(Vec3::new(0.0, -1.0, 0.0));

        let edge1 = mesh_graph.add_edge(v1, v2);
        let edge1_d = mesh_graph.add_edge(v1, v2);
        let edge2 = mesh_graph.add_edge(v2, v3);
        let edge2_d = mesh_graph.add_edge(v2, v3);

        mesh_graph.add_face_from_halfedges(edge1.start_to_end_he_id, edge2.start_to_end_he_id);
        mesh_graph.add_face_from_halfedge_and_vertex(edge2.twin_he_id, v4);

        mesh_graph.add_face_from_halfedge_and_vertex(edge2_d.start_to_end_he_id, v4);
        mesh_graph.add_face_from_halfedges(edge2_d.twin_he_id, edge1_d.twin_he_id);

        mesh_graph.add_face_from_halfedge_and_vertex(edge1_d.start_to_end_he_id, v5);

        #[cfg(feature = "rerun")]
        mesh_graph.log_rerun();

        let mut removed_vertices = Vec::new();
        let mut removed_halfedges = Vec::new();
        let mut removed_faces = Vec::new();

        mesh_graph.remove_neighboring_flaps(
            v1,
            &mut removed_vertices,
            &mut removed_halfedges,
            &mut removed_faces,
        );

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        assert_eq!(removed_vertices.len(), 0);
        assert_eq!(removed_halfedges.len(), 6);
        assert_eq!(removed_faces.len(), 2);
    }
}
