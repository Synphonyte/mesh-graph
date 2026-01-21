use hashbrown::HashSet;
use itertools::Itertools;
use tracing::instrument;

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none, utils::unwrap_or_return};

impl MeshGraph {
    /// Deletes a face from the mesh graph.
    ///
    /// It also deletes the vertices and halfedges that are no
    /// longer connected to any other faces.
    ///
    /// Returns the ids of the removed vertices and halfedges.
    #[instrument(skip(self))]
    pub fn delete_face(&mut self, face_id: FaceId) -> (Vec<VertexId>, Vec<HalfedgeId>) {
        let mut vertices = vec![];

        let halfedges = self
            .halfedges
            .iter()
            .filter_map(|(he_id, he)| {
                if he.face == Some(face_id) {
                    Some((he_id, *he))
                } else {
                    None
                }
            })
            .collect_vec();

        let mut deleted_halfedges = HashSet::with_capacity(4);

        for (he_id, he) in halfedges {
            vertices.push(he.end_vertex);

            let twin_id = unwrap_or_return!(he.twin, "Twin not found", (vec![], vec![]));

            let twin = unwrap_or_return!(
                self.halfedges.get(twin_id),
                "Twin halfedge not found",
                (vec![], vec![])
            );
            if twin.is_boundary() {
                deleted_halfedges.insert(he_id);
                deleted_halfedges.insert(twin_id);
            } else if twin.twin != Some(he_id) {
                deleted_halfedges.insert(he_id);
            } else {
                // already checked above
                self.halfedges[he_id].face = None;
                self.halfedges[he_id].next = None;
            }
        }

        #[cfg(feature = "rerun")]
        self.log_hes_rerun(
            "deleted_by_face_deletion",
            &deleted_halfedges.iter().copied().collect::<Vec<_>>(),
        );

        let mut deleted_vertices = Vec::with_capacity(3);

        for &vertex_id in &vertices {
            let mut all_deleted = true;
            for he_id in unwrap_or_return!(
                self.outgoing_halfedges.get(vertex_id),
                "Outgoing halfedges not found",
                (vec![], vec![])
            ) {
                if !deleted_halfedges.contains(he_id) {
                    all_deleted = false;
                    break;
                }
            }

            if all_deleted {
                self.delete_only_vertex(vertex_id);
                deleted_vertices.push(vertex_id);
            }
        }

        // Update connections from vertices to deleted halfedges
        for &he_id in &deleted_halfedges {
            // checked when inserted into deleted_halfedges
            let start_v_id = unwrap_or_return!(
                self.halfedges[he_id].start_vertex(self),
                "Start vertex not found",
                (vec![], vec![])
            );
            if let Some(v) = self.vertices.get(start_v_id) {
                self.vertices[start_v_id].outgoing_halfedge = v
                    .outgoing_halfedges(self)
                    .into_iter()
                    .find(|id| !deleted_halfedges.contains(id))
                    .or_else(error_none!("No new outgoing halfedge found"));
            }
        }

        for he_id in &deleted_halfedges {
            self.halfedges.remove(*he_id);

            for &v_id in &vertices {
                if let Some(outgoing_hes) = self.outgoing_halfedges.get_mut(v_id) {
                    outgoing_hes.retain(|id| id != he_id);
                }
            }
        }

        // already checked at the start of the function
        self.bvh.remove(self.faces[face_id].index);
        self.faces.remove(face_id);

        (deleted_vertices, Vec::from_iter(deleted_halfedges))
    }

    /// Deletes only a vertex, without deleting any faces or halfedges connected to it.
    pub fn delete_only_vertex(&mut self, vertex_id: VertexId) {
        self.outgoing_halfedges.remove(vertex_id);
        self.positions.remove(vertex_id);
        if let Some(normals) = &mut self.vertex_normals {
            normals.remove(vertex_id);
        }
        self.vertices.remove(vertex_id);
    }

    /// Deletes only a halfedge, without deleting any faces or vertices connected to it.
    /// This also doesn't delete or update any twin
    pub fn delete_only_halfedge(&mut self, he_id: HalfedgeId) {
        if let Some(he) = self.halfedges.get(he_id) {
            if let Some(start_v_id) = he.start_vertex(self)
                && let Some(hes) = self.outgoing_halfedges.get_mut(start_v_id)
            {
                hes.retain(|out_he_id| *out_he_id != he_id);
            }

            self.halfedges.remove(he_id);
        }
    }

    pub fn delete_only_halfedge_and_twin(&mut self, he_id: HalfedgeId) {
        if let Some(he) = self.halfedges.get(he_id) {
            if let Some(start_v_id) = he.start_vertex(self)
                && let Some(hes) = self.outgoing_halfedges.get_mut(start_v_id)
            {
                hes.retain(|out_he_id| *out_he_id != he_id);
            }

            if let Some(twin_he_id) = he.twin {
                self.delete_only_halfedge(twin_he_id);
            }

            self.halfedges.remove(he_id);
        }
    }
}


#[cfg(test)]
mod tests {
    use glam::{Mat4, vec3};

    use crate::{utils::{extend_with, get_tracing_subscriber}, *};

    #[test]
    fn test_delete_face() {
        get_tracing_subscriber();
        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 0.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.5, 0.0);
        let p_3 = vec3(-1.0, -0.5, 0.0);
        let p_4 = vec3(0.0, -1.0, 0.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6 = vec3(1.0, 0.5, 0.0);

        let points = vec![p_c, p_1, p_2, p_3, p_4, p_5, p_6];
        let vertex_ids = extend_with(&mut meshgraph, &points.clone(), Mat4::default(), 5.0, 0);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        let face_count = meshgraph.faces.len();
        let face_id = meshgraph.vertices[vertex_ids[6]].faces(&meshgraph).next().unwrap();
        meshgraph.delete_face(face_id);
        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        assert_eq!(meshgraph.faces.len(), face_count - 1);
    }
}
