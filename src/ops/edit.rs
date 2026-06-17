use glam::Vec3;
use hashbrown::{HashMap, HashSet};
use itertools::Itertools;
use slotmap::SparseSecondaryMap;
use tracing::{error, instrument};

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none, utils::unwrap_or_return};

pub struct MergeVertices {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,
}

impl MeshGraph {
    /// Merges the given vertices into a single vertex, reconnecting halfedges and faces as needed.
    #[instrument(skip_all)]
    pub fn merge_vertices(
        &mut self,
        vertices: impl IntoIterator<Item = VertexId>,
    ) -> MergeVertices {
        let vertex_ids: Vec<VertexId> = vertices
            .into_iter()
            .filter(|v| self.vertices.contains_key(*v))
            .unique()
            .collect();

        if vertex_ids.len() < 2 {
            return MergeVertices {
                removed_vertices: Vec::new(),
                removed_halfedges: Vec::new(),
                removed_faces: Vec::new(),
            };
        }

        let vertex_set: HashSet<VertexId> = vertex_ids.iter().copied().collect();
        let survivor_id = vertex_ids[0];

        // Compute average position
        let mut avg_pos = Vec3::ZERO;
        let mut count = 0.0;
        for &v_id in &vertex_ids {
            if let Some(&pos) = self.positions.get(v_id) {
                avg_pos += pos;
                count += 1.0;
            }
        }
        if count > 0.0 {
            avg_pos /= count;
        }

        // Find faces to remove: any face with 2+ vertices in the merge set becomes degenerate
        let mut faces_to_remove = HashSet::new();
        for &v_id in &vertex_ids {
            for face_id in self.vertex_adjacent_faces(v_id) {
                if faces_to_remove.contains(&face_id) {
                    continue;
                }
                if let Some(face) = self.faces.get(face_id) {
                    let merged_count = face
                        .vertices(self)
                        .filter(|v| vertex_set.contains(v))
                        .count();
                    if merged_count >= 2 {
                        faces_to_remove.insert(face_id);
                    }
                }
            }
        }

        // Determine which halfedges to remove vs make boundary
        let mut halfedges_to_remove = HashSet::new();
        for &face_id in &faces_to_remove {
            let face_hes: Vec<_> = self
                .halfedges
                .iter()
                .filter_map(|(he_id, he)| {
                    if he.face == Some(face_id) {
                        Some((he_id, *he))
                    } else {
                        None
                    }
                })
                .collect();

            for (he_id, he) in face_hes {
                if let Some(twin_id) = he.twin {
                    if let Some(twin) = self.halfedges.get(twin_id) {
                        if twin.is_boundary()
                            || twin.face.map_or(false, |f| faces_to_remove.contains(&f))
                        {
                            halfedges_to_remove.insert(he_id);
                            halfedges_to_remove.insert(twin_id);
                        } else {
                            if let Some(he_mut) = self.halfedges.get_mut(he_id) {
                                he_mut.face = None;
                                he_mut.next = None;
                            }
                        }
                    }
                }
            }
        }

        // Remove faces from BVH and faces slotmap
        for &face_id in &faces_to_remove {
            if let Some(face) = self.faces.get(face_id) {
                self.bvh.remove(face.index);
            }
            self.faces.remove(face_id);
        }

        // Collect outgoing halfedge cleanup info before removing halfedges
        // (start_vertex lookup needs the twin to be present)
        let mut outgoing_cleanup: Vec<(VertexId, HalfedgeId)> = Vec::new();
        for &he_id in &halfedges_to_remove {
            if let Some(he) = self.halfedges.get(he_id) {
                if let Some(start_v_id) = he.start_vertex(self) {
                    outgoing_cleanup.push((start_v_id, he_id));
                }
            }
        }

        for (v_id, he_id) in outgoing_cleanup {
            if let Some(out_hes) = self.outgoing_halfedges.get_mut(v_id) {
                out_hes.retain(|id| *id != he_id);
            }
        }

        // Remove halfedges
        for &he_id in &halfedges_to_remove {
            self.halfedges.remove(he_id);
        }

        let mut removed_vertices = Vec::with_capacity(vertex_ids.len() - 1);

        // Merge non-survivor vertices into survivor
        for &v_id in &vertex_ids[1..] {
            if !self.vertices.contains_key(v_id) {
                continue;
            }

            removed_vertices.push(v_id);

            // Get incoming halfedges to v_id (twins of outgoing halfedges from v_id)
            let incoming_he_ids: Vec<HalfedgeId> = self
                .outgoing_halfedges
                .get(v_id)
                .map(|out_hes| {
                    out_hes
                        .iter()
                        .filter_map(|he_id| self.halfedges.get(*he_id)?.twin)
                        .collect()
                })
                .unwrap_or_default();

            for he_id in incoming_he_ids {
                if let Some(he) = self.halfedges.get_mut(he_id) {
                    he.end_vertex = survivor_id;
                }
            }

            // Transfer outgoing halfedges to survivor
            let outgoing = self
                .outgoing_halfedges
                .get(v_id)
                .cloned()
                .unwrap_or_default();

            if let Some(entry) = self.outgoing_halfedges.entry(survivor_id) {
                entry.or_default().extend(outgoing);
            }

            self.remove_only_vertex(v_id);
        }

        let mut removed_halfedges: Vec<HalfedgeId> = halfedges_to_remove.into_iter().collect();
        let removed_faces: Vec<FaceId> = faces_to_remove.into_iter().collect();

        // Check if survivor still exists (could have been removed if all faces were removed)
        if !self.vertices.contains_key(survivor_id) {
            return MergeVertices {
                removed_vertices,
                removed_halfedges,
                removed_faces,
            };
        }

        self.positions[survivor_id] = avg_pos;

        // Clean up stale outgoing halfedges for survivor
        if let Some(out_hes) = self.outgoing_halfedges.get_mut(survivor_id) {
            out_hes.retain(|he_id| self.halfedges.contains_key(*he_id));
        }

        // Remove duplicate halfedges that now share the same edge after vertex merge
        {
            let out_hes = self
                .outgoing_halfedges
                .get(survivor_id)
                .cloned()
                .unwrap_or_default();

            let mut by_end: HashMap<VertexId, Vec<HalfedgeId>> = HashMap::new();
            for &he_id in &out_hes {
                if let Some(he) = self.halfedges.get(he_id) {
                    by_end.entry(he.end_vertex).or_default().push(he_id);
                }
            }

            let mut affected_vertices = HashSet::new();

            for (end_v, group) in by_end {
                if group.len() < 2 {
                    continue;
                }

                // Pick best forward halfedge (prefer non-boundary)
                let best_fwd = group
                    .iter()
                    .copied()
                    .max_by_key(|&id| {
                        self.halfedges
                            .get(id)
                            .map_or(0u8, |h| if h.face.is_some() { 1 } else { 0 })
                    })
                    .unwrap();

                // Collect valid twins
                let twins: Vec<HalfedgeId> = group
                    .iter()
                    .filter_map(|&id| {
                        self.halfedges
                            .get(id)?
                            .twin
                            .filter(|t| self.halfedges.contains_key(*t))
                    })
                    .collect();

                // Pick best reverse halfedge (prefer non-boundary)
                let best_rev = twins.iter().copied().max_by_key(|&id| {
                    self.halfedges
                        .get(id)
                        .map_or(0u8, |h| if h.face.is_some() { 1 } else { 0 })
                });

                // Update twin pointers
                if let Some(rev_id) = best_rev {
                    if let Some(he) = self.halfedges.get_mut(best_fwd) {
                        he.twin = Some(rev_id);
                    }
                    if let Some(he) = self.halfedges.get_mut(rev_id) {
                        he.twin = Some(best_fwd);
                    }
                }

                // Remove duplicate forward halfedges
                for &he_id in &group {
                    if he_id != best_fwd {
                        self.halfedges.remove(he_id);
                        removed_halfedges.push(he_id);
                    }
                }

                // Remove duplicate reverse halfedges
                for &twin_id in &twins {
                    if Some(twin_id) != best_rev {
                        self.halfedges.remove(twin_id);
                        removed_halfedges.push(twin_id);
                    }
                }

                affected_vertices.insert(end_v);
            }

            // Clean up outgoing halfedges for survivor
            if let Some(out_hes) = self.outgoing_halfedges.get_mut(survivor_id) {
                out_hes.retain(|he_id| self.halfedges.contains_key(*he_id));
            }

            // Clean up outgoing halfedges for affected adjacent vertices
            for v in affected_vertices {
                if let Some(out_hes) = self.outgoing_halfedges.get_mut(v) {
                    out_hes.retain(|he_id| self.halfedges.contains_key(*he_id));
                }
                let new_out = self
                    .outgoing_halfedges
                    .get(v)
                    .and_then(|hes| hes.first().copied());
                if let Some(vertex) = self.vertices.get_mut(v) {
                    vertex.outgoing_halfedge = new_out;
                }
            }
        }

        // Update outgoing_halfedge reference
        let new_outgoing = self
            .outgoing_halfedges
            .get(survivor_id)
            .and_then(|hes| hes.first().copied());

        if let Some(vertex) = self.vertices.get_mut(survivor_id) {
            vertex.outgoing_halfedge = new_outgoing;
        }

        self.make_outgoing_halfedge_boundary_if_possible(survivor_id);
        self.compute_vertex_normal(survivor_id);

        MergeVertices {
            removed_vertices,
            removed_halfedges,
            removed_faces,
        }
    }

    /// Flips this edge so that it represents the other diagonal described by the quad formed by the two incident triangles.
    ///
    /// ```text
    ///     *                    *
    ///    / \                  / \
    ///   /   \                / ‖ \
    ///  /     \              /  ‖  \
    /// * ===== *     =>     *   ‖   *
    ///  \     /              \  ‖  /
    ///   \   /                \ ‖ /
    ///    \ /                  \ /
    ///     *                    *
    /// ```
    #[instrument(skip(self))]
    pub fn flip_edge(&mut self, halfedge_id: HalfedgeId) {
        #[cfg(feature = "rerun")]
        self.log_he_rerun("flip", halfedge_id);

        let he = unwrap_or_return!(self.halfedges.get(halfedge_id), "Halfedge not found");

        let prev_he_id = unwrap_or_return!(he.prev(self), "Prev not found");
        let prev_he = unwrap_or_return!(self.halfedges.get(prev_he_id), "Prev not found");
        let start_v_id = prev_he.end_vertex;
        let prev_twin_he_id = unwrap_or_return!(prev_he.twin, "Prev twin not found");

        let next_he_id = unwrap_or_return!(he.next, "Next not found");
        let next_he = unwrap_or_return!(self.halfedges.get(next_he_id), "Next not found");
        let opposite_v_id = next_he.end_vertex;
        let next_twin_he_id = unwrap_or_return!(next_he.twin, "Next twin not found");

        let twin_he_id = unwrap_or_return!(he.twin, "Twin not found");
        let twin_he = unwrap_or_return!(self.halfedges.get(twin_he_id), "Twin not found");

        let twin_prev_he_id = unwrap_or_return!(twin_he.prev(self), "Prev not found");
        let twin_prev_he = unwrap_or_return!(self.halfedges.get(twin_prev_he_id), "Prev not found");
        let twin_start_v_id = twin_prev_he.end_vertex;
        let twin_prev_twin_he_id = unwrap_or_return!(twin_prev_he.twin, "Prev twin twin not found");

        let twin_next_he_id = unwrap_or_return!(twin_he.next, "Next not found");
        let twin_next_he = unwrap_or_return!(self.halfedges.get(twin_next_he_id), "Next not found");
        let twin_opposite_v_id = twin_next_he.end_vertex;
        let twin_next_twin_he_id = unwrap_or_return!(twin_next_he.twin, "Next twin twin not found");

        self.halfedges[halfedge_id].end_vertex = opposite_v_id;

        self.halfedges[prev_he_id].end_vertex = twin_opposite_v_id;
        self.make_twins(prev_he_id, twin_next_twin_he_id);
        self.halfedges[next_he_id].end_vertex = start_v_id;
        self.make_twins(next_he_id, prev_twin_he_id);

        self.remove_outgoing_halfedge(start_v_id, halfedge_id);
        self.remove_outgoing_halfedge(start_v_id, twin_next_he_id);
        self.add_outgoing_halfedge(start_v_id, prev_he_id);

        self.remove_outgoing_halfedge(opposite_v_id, prev_he_id);
        self.add_outgoing_halfedge(opposite_v_id, next_he_id);
        self.add_outgoing_halfedge(opposite_v_id, twin_he_id);

        self.halfedges[twin_he_id].end_vertex = twin_opposite_v_id;

        self.halfedges[twin_prev_he_id].end_vertex = opposite_v_id;
        self.make_twins(twin_prev_he_id, next_twin_he_id);
        self.halfedges[twin_next_he_id].end_vertex = twin_start_v_id;
        self.make_twins(twin_next_he_id, twin_prev_twin_he_id);

        self.remove_outgoing_halfedge(twin_start_v_id, twin_he_id);
        self.remove_outgoing_halfedge(twin_start_v_id, next_he_id);
        self.add_outgoing_halfedge(twin_start_v_id, twin_prev_he_id);

        self.remove_outgoing_halfedge(twin_opposite_v_id, twin_prev_he_id);
        self.add_outgoing_halfedge(twin_opposite_v_id, twin_next_he_id);
        self.add_outgoing_halfedge(twin_opposite_v_id, halfedge_id);

        // checked if halfedge exists above
        let face_id1 = unwrap_or_return!(self.halfedges[halfedge_id].face, "Face not found");
        let face1 = unwrap_or_return!(self.faces.get(face_id1), "Face not found");
        self.bvh
            .insert_or_update_partially(face1.aabb(self), face1.index, 0.0);

        // checked if halfedge exists above
        let face_id2 = unwrap_or_return!(self.halfedges[twin_he_id].face, "Face not found");
        let face2 = unwrap_or_return!(self.faces.get(face_id2), "Face not found");
        self.bvh
            .insert_or_update_partially(face2.aabb(self), face2.index, 0.0);
    }

    /// Makes two halfedges twins of each other. Doesn't change anything else
    pub fn make_twins(&mut self, he_id1: HalfedgeId, he_id2: HalfedgeId) {
        let he1 = unwrap_or_return!(self.halfedges.get_mut(he_id1), "Halfedge not found");
        he1.twin = Some(he_id2);

        let he2 = unwrap_or_return!(self.halfedges.get_mut(he_id2), "Halfedge not found");
        he2.twin = Some(he_id1);
    }

    /// Removes the outgoing halfedge from a vertex. Doesn't change anything else.
    #[instrument(skip(self))]
    pub fn remove_outgoing_halfedge(&mut self, vertex_id: VertexId, halfedge_id: HalfedgeId) {
        let outgoing_halfedges = unwrap_or_return!(
            self.outgoing_halfedges.get_mut(vertex_id),
            "No outgoing halfedges found"
        );

        outgoing_halfedges.retain(|he_id| *he_id != halfedge_id);
    }

    /// Adds the outgoing halfedge to a vertex and overrides the vertex.outgoing_halfedge
    #[instrument(skip(self))]
    pub fn add_outgoing_halfedge(&mut self, vertex_id: VertexId, outgoing_halfedge: HalfedgeId) {
        let vertex = unwrap_or_return!(self.vertices.get_mut(vertex_id), "Vertex not found");
        vertex.outgoing_halfedge = Some(outgoing_halfedge);

        let v_outgoing_halfedges = unwrap_or_return!(
            self.outgoing_halfedges.get_mut(vertex_id),
            "No outgoing halfedges found"
        );
        v_outgoing_halfedges.push(outgoing_halfedge);
    }

    /// Smooths the position of the vertex by computing the average of its own and its neighbors' positions and
    /// moving it there. Also called Laplacian Smoothing.
    #[instrument(skip_all)]
    pub fn smooth_vertices(&mut self, vertices: impl IntoIterator<Item = VertexId>) {
        let mut new_positions = SparseSecondaryMap::new();

        let mut affected_face_ids = HashSet::new();

        for vertex_id in vertices.into_iter() {
            let Some(pos) = self.compute_smoothed_vertex_pos(vertex_id) else {
                continue;
            };

            new_positions.insert(vertex_id, pos);

            // vertex checked if exists in `compute_smoothed_vertex_pos()`
            affected_face_ids.extend(self.vertices[vertex_id].faces(self));
        }

        for (vertex_id, &pos) in &new_positions {
            self.positions.insert(vertex_id, pos);
        }

        for vertex_id in new_positions.keys() {
            self.compute_vertex_normal(vertex_id);
        }

        for face_id in affected_face_ids {
            let Some(face) = self.faces.get(face_id) else {
                error!("Face {:?} does not exist", face_id);
                continue;
            };

            self.bvh
                .insert_or_update_partially(face.aabb(self), face.index, 0.0);
        }
    }

    /// Smooth the position of the vertex by computing the average of its own and its neighbors' positions and
    /// moving it there. Also called Laplacian Smoothing.
    ///
    /// > Note: If you want to smooth multiple vertices, use the `smooth_vertices` method instead of
    /// > calling this method multiple times.
    #[instrument(skip(self))]
    pub fn smooth_vertex(&mut self, vertex_id: VertexId) {
        let pos = unwrap_or_return!(
            self.compute_smoothed_vertex_pos(vertex_id),
            "Couldn't compute smoothed position"
        );

        self.positions.insert(vertex_id, pos);
        self.compute_vertex_normal(vertex_id);

        // vertex checked if exists in `compute_smoothed_vertex_pos()`
        for face_id in self.vertices[vertex_id].faces(self).collect_vec() {
            let Some(face) = self.faces.get(face_id) else {
                error!("Face {:?} does not exist", face_id);
                continue;
            };

            self.bvh
                .insert_or_update_partially(face.aabb(self), face.index, 0.0);
        }
    }

    #[instrument(skip(self))]
    fn compute_smoothed_vertex_pos(&mut self, vertex_id: VertexId) -> Option<Vec3> {
        let vertex = self.vertices.get(vertex_id)?;

        let mut pos = *self
            .positions
            .get(vertex_id)
            .or_else(error_none!("Position not found for id {vertex_id:?}"))?;

        let mut count = 1.0;
        for neighbor_v_id in vertex.neighbours(self) {
            let neighbor_pos = *self.positions.get(neighbor_v_id).or_else(error_none!(
                "Neighbor position not found for id {neighbor_v_id:?}"
            ))?;

            pos += neighbor_pos;
            count += 1.0;
        }

        pos /= count;

        Some(pos)
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;

    #[cfg(feature = "gltf")]
    #[test]
    fn test_merge_vertices_cube() {
        use crate::{integrations::gltf, utils::get_tracing_subscriber};

        get_tracing_subscriber();
        let mut meshgraph = gltf::load("src/ops/glb/cube.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let x0_vertices: Vec<VertexId> = meshgraph
            .positions
            .iter()
            .filter_map(|(v_id, pos)| if pos.x == 0.0 { Some(v_id) } else { None })
            .collect();

        let x0_count = x0_vertices.len();
        assert!(x0_count >= 2, "Expected at least 2 vertices with x=0");

        let MergeVertices {
            removed_vertices,
            removed_halfedges,
            removed_faces,
        } = meshgraph.merge_vertices(x0_vertices);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        let remaining_x0: Vec<_> = meshgraph
            .positions
            .iter()
            .filter(|(_, pos)| pos.x == 0.0)
            .collect();

        assert_eq!(remaining_x0.len(), 1);

        assert_eq!(meshgraph.vertices.len(), 5);
        assert_eq!(meshgraph.halfedges.len(), 18);
        assert_eq!(meshgraph.faces.len(), 6);

        assert_eq!(removed_vertices.len(), 3);
        assert_eq!(removed_halfedges.len(), 18);
        assert_eq!(removed_faces.len(), 6);
    }

    #[cfg(feature = "gltf")]
    #[test]
    fn test_merge_vertices_cube_w_missing_triangle() {
        use crate::{integrations::gltf, utils::get_tracing_subscriber};

        get_tracing_subscriber();
        let mut meshgraph = gltf::load("src/ops/glb/cube_w_missing_triangle.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let x0_vertices: Vec<VertexId> = meshgraph
            .positions
            .iter()
            .filter_map(|(v_id, pos)| if pos.x == 0.0 { Some(v_id) } else { None })
            .collect();

        let x0_count = x0_vertices.len();
        assert!(x0_count >= 2, "Expected at least 2 vertices with x=0");

        let MergeVertices {
            removed_vertices,
            removed_halfedges,
            removed_faces,
        } = meshgraph.merge_vertices(x0_vertices);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        let remaining_x0: Vec<_> = meshgraph
            .positions
            .iter()
            .filter(|(_, pos)| pos.x == 0.0)
            .collect();

        assert_eq!(remaining_x0.len(), 1);

        assert_eq!(meshgraph.vertices.len(), 5);
        assert_eq!(meshgraph.halfedges.len(), 18);
        assert_eq!(meshgraph.faces.len(), 6);

        assert_eq!(removed_vertices.len(), 3);
        assert_eq!(removed_halfedges.len(), 18);
        assert_eq!(removed_faces.len(), 5);
    }

    #[test]
    fn test_flip_edge() {
        let mut mesh_graph = MeshGraph::new();

        let v_id1 = mesh_graph.add_vertex(Vec3::new(-1.0, 1.0, 0.0));
        let v_id2 = mesh_graph.add_vertex(Vec3::new(-1.0, -1.0, 0.0));
        let v_id3 = mesh_graph.add_vertex(Vec3::new(1.0, -1.0, 0.0));
        let v_id4 = mesh_graph.add_vertex(Vec3::new(1.0, 1.0, 0.0));

        mesh_graph.add_face_from_vertices(v_id1, v_id2, v_id3);
        mesh_graph.add_face_from_vertices(v_id1, v_id3, v_id4);

        #[cfg(feature = "rerun")]
        mesh_graph.log_rerun();

        assert!(mesh_graph.halfedge_from_to(v_id2, v_id4).is_none());

        assert_eq!(mesh_graph.outgoing_halfedges[v_id1].len(), 3);
        assert_eq!(mesh_graph.outgoing_halfedges[v_id2].len(), 2);
        assert_eq!(mesh_graph.outgoing_halfedges[v_id3].len(), 3);
        assert_eq!(mesh_graph.outgoing_halfedges[v_id4].len(), 2);

        mesh_graph.flip_edge(mesh_graph.halfedge_from_to(v_id1, v_id3).unwrap());

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        assert!(mesh_graph.halfedge_from_to(v_id1, v_id3).is_none());
        assert!(mesh_graph.halfedge_from_to(v_id2, v_id4).is_some());

        assert_eq!(mesh_graph.outgoing_halfedges[v_id1].len(), 2);
        assert_eq!(mesh_graph.outgoing_halfedges[v_id2].len(), 3);
        assert_eq!(mesh_graph.outgoing_halfedges[v_id3].len(), 2);
        assert_eq!(mesh_graph.outgoing_halfedges[v_id4].len(), 3);
    }
}
