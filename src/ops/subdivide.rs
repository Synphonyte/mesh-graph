use hashbrown::{HashMap, HashSet};

use crate::{HalfedgeId, MeshGraph, Selection, SelectionOps, VertexId};

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
    pub fn subdivide_edge(&mut self, halfedge_id: HalfedgeId) -> Vec<HalfedgeId> {
        let he = &self.halfedges[halfedge_id];
        let twin_id = he.twin();

        let start_v = he.start_vertex(self);
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
            normals[center_v] = (normals[start_v] + normals[end_v]).normalize();
        }

        let new_he = self.insert_halfedge(end_v);
        self.vertices[center_v].outgoing_halfedge = Some(new_he);

        let mut new_halfedges = Vec::with_capacity(3);
        new_halfedges.push(new_he);

        if let Some(new_face_he) = self.subdivide_face(halfedge_id, new_he, center_v) {
            new_halfedges.push(new_face_he);
        }

        let new_twin = self.insert_halfedge(start_v);

        if let Some(new_face_he) = self.subdivide_face(twin_id, new_twin, center_v) {
            new_halfedges.push(new_face_he);
        }

        self.halfedges[new_he].twin = Some(twin_id);
        self.halfedges[twin_id].twin = Some(new_he);

        self.halfedges[halfedge_id].twin = Some(new_twin);
        self.halfedges[new_twin].twin = Some(halfedge_id);

        // self.vertices[end_v].outgoing_halfedge = Some(new_twin);
        // self.vertices[start_v].outgoing_halfedge = Some(new_he);

        new_halfedges
    }

    /// Subdivides a triangle into two halves. Used in [Self::subdivide_edge].
    fn subdivide_face(
        &mut self,
        existing_halfedge_id: HalfedgeId,
        new_halfedge_id: HalfedgeId,
        center_v: VertexId,
    ) -> Option<HalfedgeId> {
        let he = &self.halfedges[existing_halfedge_id];

        let face_id = he.face?;
        self.faces[face_id].halfedge = existing_halfedge_id;

        let next_he = self.halfedges[existing_halfedge_id].next.unwrap();
        let last_he = self.halfedges[next_he].next.unwrap();

        // rewire existing face
        let new_he = self.insert_halfedge(self.halfedges[next_he].end_vertex);

        self.halfedges[existing_halfedge_id].next = Some(new_he);
        self.halfedges[new_he].next = Some(last_he);
        self.halfedges[new_he].face = Some(face_id);

        // insert new face
        let new_face_id = self.insert_face(new_halfedge_id);

        self.qbvh.pre_update_or_insert(self.faces[face_id]);
        self.qbvh.pre_update_or_insert(self.faces[new_face_id]);

        let new_twin = self.insert_halfedge(center_v);

        self.halfedges[new_twin].next = Some(new_halfedge_id);
        self.halfedges[new_twin].face = Some(new_face_id);
        self.halfedges[new_twin].twin = Some(new_he);
        self.halfedges[new_he].twin = Some(new_twin);

        self.halfedges[new_halfedge_id].next = Some(next_he);
        self.halfedges[new_halfedge_id].face = Some(new_face_id);

        self.halfedges[next_he].next = Some(new_twin);
        self.halfedges[next_he].face = Some(new_face_id);

        self.halfedges[existing_halfedge_id].end_vertex = center_v;

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
    pub fn subdivide_until_edges_below_max_length(
        &mut self,
        max_length_squared: f32,
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

        let mut halfedges_to_subdivide = dedup_halfedges
            .into_iter()
            .filter_map(|he| {
                let len = self.halfedges[he].length_squared(self);
                (len > max_length_squared).then_some((he, len))
            })
            .collect::<HashMap<_, _>>();

        while !halfedges_to_subdivide.is_empty() {
            let mut max_len = 0.0;
            let mut max_he = HalfedgeId::default();

            for (&he, &len) in &halfedges_to_subdivide {
                if len > max_len {
                    max_len = len;
                    max_he = he;
                }
            }

            halfedges_to_subdivide.remove(&max_he);

            let new_edges = self.subdivide_edge(max_he);

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

            for new_edge in new_edges {
                selection.insert(new_edge);
                if let Some(new_twin) = self.halfedges[new_edge].twin {
                    selection.insert(new_twin);
                }

                let len_sqr = self.halfedges[new_edge].length_squared(self);
                if len_sqr > max_length_squared {
                    #[cfg(feature = "rerun")]
                    {
                        self.log_he_rerun("/subdivide/new_edge", new_edge);
                    }
                    halfedges_to_subdivide.insert(new_edge, len_sqr);
                }
            }

            let len_sqr = self.halfedges[max_he].length_squared(self);
            if len_sqr > max_length_squared {
                #[cfg(feature = "rerun")]
                {
                    self.log_he_rerun("/subdivide/prev_edge", max_he);
                }
                halfedges_to_subdivide.insert(max_he, len_sqr);
            }
        }
    }
}

#[cfg(test)]
mod tests {}
