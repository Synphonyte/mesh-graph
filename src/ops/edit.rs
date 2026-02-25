use glam::Vec3;
use hashbrown::HashSet;
use itertools::Itertools;
use slotmap::SparseSecondaryMap;
use tracing::{error, instrument};

use crate::{HalfedgeId, MeshGraph, VertexId, error_none, utils::unwrap_or_return};

impl MeshGraph {
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
