use hashbrown::{HashMap, HashSet};
use tracing::{error, instrument};

use crate::{
    HalfedgeId, MeshGraph, Selection, SelectionOps, VertexId, error_none, utils::unwrap_or_return,
};

#[cfg(feature = "rerun")]
use crate::utils::vec3_array;

impl MeshGraph {
    /// Subdivides an edge by computing it's center vertex. This also subdivides any adjacent triangles and
    /// makes sure everything is properly reconnected. Works only on triangle meshes.
    ///
    /// Returns the id of the new halfedge which goes from the center vertex to the original edge's end vertex.
    /// And also return the halfedges that are created by subdividing the adjacent faces. Only one of the two twin
    /// halfedges per face subdivision is returned. In total the number `n` of halfedges returned is `1 <= n <= 3`.
    /// (The one from dividing the halfedge and at most 2 from dividing the two adjacent faces).
    #[instrument(skip(self))]
    pub fn subdivide_edge(&mut self, halfedge_id: HalfedgeId) -> Vec<HalfedgeId> {
        let mut new_halfedges = Vec::with_capacity(3);

        let he = unwrap_or_return!(
            self.halfedges.get(halfedge_id),
            "Halfedge not found",
            new_halfedges
        );
        let twin_id = unwrap_or_return!(he.twin, "Twin halfedge not found", new_halfedges);

        let start_v = unwrap_or_return!(
            he.start_vertex(self),
            "Start vertex not found",
            new_halfedges
        );
        let end_v = he.end_vertex;

        let start_pos = self.positions[start_v];
        let end_pos = self.positions[end_v];

        let center_pos = (start_pos + end_pos) * 0.5;

        #[cfg(feature = "rerun")]
        {
            crate::RR
                .log(
                    "meshgraph/subdivide/edge",
                    &rerun::Arrows3D::from_vectors([vec3_array(end_pos - start_pos)])
                        .with_origins([vec3_array(start_pos)]),
                )
                .unwrap();

            crate::RR
                .log(
                    "meshgraph/subdivide/center",
                    &rerun::Points3D::new([vec3_array(center_pos)]),
                )
                .unwrap();
        }

        let center_v = self.insert_vertex(center_pos);
        if let Some(normals) = &mut self.vertex_normals {
            let start_normal = unwrap_or_return!(
                normals.get(start_v),
                "Start normal not found",
                new_halfedges
            );
            let end_normal =
                unwrap_or_return!(normals.get(end_v), "End normal not found", new_halfedges);
            normals[center_v] = (start_normal + end_normal).normalize();
        }

        let new_he = self.insert_halfedge(center_v, end_v);
        // inserted just above
        self.vertices[center_v].outgoing_halfedge = Some(new_he);

        new_halfedges.push(new_he);

        if let Some(new_face_he) = self.subdivide_face(halfedge_id, new_he, center_v) {
            new_halfedges.push(new_face_he);
        }

        let new_twin = self.insert_halfedge(center_v, start_v);

        if let Some(new_face_he) = self.subdivide_face(twin_id, new_twin, center_v) {
            new_halfedges.push(new_face_he);
        }

        // inserted above
        self.halfedges[new_he].twin = Some(twin_id);
        unwrap_or_return!(
            self.halfedges.get_mut(twin_id),
            "Twin halfedge not found",
            new_halfedges
        )
        .twin = Some(new_he);

        // checked in the beginning of the function
        self.halfedges[halfedge_id].twin = Some(new_twin);
        // inserted above
        self.halfedges[new_twin].twin = Some(halfedge_id);

        // self.vertices[end_v].outgoing_halfedge = Some(new_twin);
        // self.vertices[start_v].outgoing_halfedge = Some(new_he);

        new_halfedges
    }

    /// Subdivides a triangle into two halves. Used in [Self::subdivide_edge].
    #[instrument(skip(self))]
    fn subdivide_face(
        &mut self,
        existing_halfedge_id: HalfedgeId,
        new_halfedge_id: HalfedgeId,
        center_v: VertexId,
    ) -> Option<HalfedgeId> {
        let he = self
            .halfedges
            .get(existing_halfedge_id)
            .or_else(error_none!("Halfedge not found"))?;

        let face_id = he.face?;
        self.faces
            .get_mut(face_id)
            .or_else(error_none!("Facee not found"))?
            .halfedge = existing_halfedge_id;

        // checked above
        let next_he = self.halfedges[existing_halfedge_id]
            .next
            .or_else(error_none!("Next halfedge missing"))?;
        let last_he = self
            .halfedges
            .get(next_he)
            .or_else(error_none!("Next halfedge not found"))?
            .next
            .or_else(error_none!("Last halfedge not found"))?;

        // rewire existing face
        let new_he = self.insert_halfedge(center_v, self.halfedges[next_he].end_vertex);

        self.halfedges[existing_halfedge_id].next = Some(new_he);
        self.halfedges[new_he].next = Some(last_he);
        self.halfedges[new_he].face = Some(face_id);

        let new_twin = self.insert_halfedge(self.halfedges[next_he].end_vertex, center_v);

        // insert new face
        let new_face_id = self.insert_face(new_halfedge_id, next_he, new_twin);

        self.halfedges[new_twin].twin = Some(new_he);
        self.halfedges[new_he].twin = Some(new_twin);

        self.halfedges[existing_halfedge_id].end_vertex = center_v;

        // checked above
        let face = self.faces[face_id];
        // freshly inserted
        let new_face = self.faces[new_face_id];
        self.bvh
            .insert_or_update_partially(face.aabb(self), face.index, 0.0);
        self.bvh
            .insert_or_update_partially(new_face.aabb(self), new_face.index, 0.0);

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("subdivide/new_he", new_he);
            self.log_he_rerun("subdivide/new_twin", new_twin);
        }

        Some(new_he)
    }

    /// Subdivide all edges in the selection until all of them are <= max_length.
    /// Please note that you have to provide the squared value of max_length.
    /// All new edges created during this process are added to the selection.
    ///
    /// This will schedule necessary updates to the QBVH but you have to call
    /// `refit()` and maybe `rebalance()` after the operation.
    #[instrument(skip(self))]
    pub fn subdivide_until_edges_below_max_length(
        &mut self,
        max_length_squared: f32,
        selection: &mut Selection,
    ) {
        let mut dedup_halfedges = HashSet::new();

        for he_id in selection.resolve_to_halfedges(self) {
            let he = unwrap_or_return!(self.halfedges.get(he_id), "Halfedge not found");
            let twin_already_in = he
                .twin
                .map(|twin| dedup_halfedges.contains(&twin))
                .unwrap_or_else(|| {
                    error!("Twin missing");
                    false
                });

            if !twin_already_in {
                dedup_halfedges.insert(he_id);
            }
        }

        let mut halfedges_to_subdivide = dedup_halfedges
            .into_iter()
            .filter_map(|he| {
                // already checked above
                let len = self.halfedges[he].length_squared(self);
                (len > max_length_squared).then_some((he, len))
            })
            .collect::<HashMap<_, _>>();

        while !halfedges_to_subdivide.is_empty() {
            let mut max_len = 0.0;
            let mut max_he_id = HalfedgeId::default();

            #[cfg(feature = "rerun")]
            self.log_hes_rerun(
                "subdivide/selection",
                &halfedges_to_subdivide
                    .iter()
                    .map(|(he, _)| *he)
                    .collect::<Vec<_>>(),
            );

            for (&he, &len) in &halfedges_to_subdivide {
                if len > max_len {
                    max_len = len;
                    max_he_id = he;
                }
            }

            halfedges_to_subdivide.remove(&max_he_id);

            let mut affected_faces = Selection::default();

            // already checked
            let max_he = self.halfedges[max_he_id];
            if let Some(face_id) = max_he.face {
                selection.insert(face_id);
                affected_faces.insert(face_id);

                #[cfg(feature = "rerun")]
                self.log_face_rerun("subdivide/selected_new", face_id);
            }

            if let Some(twin_id) = max_he.twin.or_else(error_none!("Twin missing"))
                && let Some(twin_he) = self
                    .halfedges
                    .get(twin_id)
                    .or_else(error_none!("Halfedge not found"))
                && let Some(twin_face_id) = twin_he.face
            {
                selection.insert(twin_face_id);
                affected_faces.insert(twin_face_id);

                #[cfg(feature = "rerun")]
                self.log_face_rerun("subdivide/selected_new", twin_face_id);
            }

            let new_edges = self.subdivide_edge(max_he_id);

            #[cfg(feature = "rerun")]
            {
                crate::RR
                    .log("meshgraph/subdivide", &rerun::Clear::recursive())
                    .unwrap();
                crate::RR
                    .log("meshgraph/halfedge/subdivide", &rerun::Clear::recursive())
                    .unwrap();
                crate::RR
                    .log("meshgraph/face/subdivide", &rerun::Clear::recursive())
                    .unwrap();
                self.log_rerun();
            }

            for new_he_id in new_edges {
                // newly inserted in `self.subdivide_edge`
                let new_he = self.halfedges[new_he_id];

                if let Some(face_id) = new_he.face {
                    selection.insert(face_id);
                    affected_faces.insert(face_id);
                    #[cfg(feature = "rerun")]
                    self.log_face_rerun("subdivide/selected_new", face_id);
                }

                let new_twin_id = unwrap_or_return!(new_he.twin, "New twin missing");
                let new_twin =
                    unwrap_or_return!(self.halfedges.get(new_twin_id), "New twin not found");
                if let Some(face_id) = new_twin.face {
                    selection.insert(face_id);
                    affected_faces.insert(face_id);
                    #[cfg(feature = "rerun")]
                    self.log_face_rerun("subdivide/selected_new", face_id);
                }
            }

            for he_id in affected_faces.resolve_to_halfedges(self) {
                let he = unwrap_or_return!(self.halfedges.get(he_id), "Halfedge not found");
                let twin = unwrap_or_return!(he.twin, "Twin missing");

                if halfedges_to_subdivide.contains_key(&twin) {
                    continue;
                }

                let len_sqr = he.length_squared(self);

                if len_sqr > max_length_squared {
                    #[cfg(feature = "rerun")]
                    self.log_he_rerun("/subdivide/new_edge", he_id);
                    halfedges_to_subdivide.insert(he_id, len_sqr);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {}
