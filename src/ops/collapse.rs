use glam::Vec3;
use hashbrown::HashSet;
use itertools::Itertools;
use tracing::{error, instrument};

use crate::{Face, FaceId, HalfedgeId, MeshGraph, VertexId, error_none, utils::unwrap_or_return};

impl MeshGraph {
    /// Collapses edges until all edges have a length above the minimum length.
    ///
    /// This will schedule necessary updates to the BVH but you have to call
    /// `refit_bvh()` after the operation.
    #[instrument(skip(self))]
    pub fn collapse_until_edges_above_min_length(
        &mut self,
        min_length_squared: f32,
        marked_vertices: &mut HashSet<VertexId>,
    ) {
        let mut halfedges_to_collapse = self.halfedges_map(|len_sqr| len_sqr < min_length_squared);

        while !halfedges_to_collapse.is_empty() {
            let mut min_len = f32::MAX;
            let mut min_he_id = None;
            let mut min_center = Vec3::ZERO;
            let mut min_twin_id = HalfedgeId::default();
            let mut min_start_v_id = VertexId::default();
            let mut min_end_v_id = VertexId::default();

            for (&he_id, &len) in &halfedges_to_collapse {
                if len < min_len
                    && let Some((twin_id, start_v_id, end_v_id, center)) =
                        self.can_collapse_edge_inner(he_id)
                {
                    min_len = len;
                    min_he_id = Some(he_id);
                    min_twin_id = twin_id;
                    min_start_v_id = start_v_id;
                    min_end_v_id = end_v_id;
                    min_center = center;
                }
            }

            let Some(min_he_id) = min_he_id else {
                // couldn't find a valid halfedge to collapse
                break;
            };

            let start_vertex_id = unwrap_or_return!(
                self.halfedges[min_he_id].start_vertex(self),
                "Start vertex not found"
            );

            let collapse_edge_result = self.collapse_edge_inner(
                min_he_id,
                min_twin_id,
                min_start_v_id,
                min_end_v_id,
                min_center,
            );

            let vertex_neighborhoods_to_check = if collapse_edge_result.added_vertices.is_empty() {
                vec![start_vertex_id]
            } else {
                marked_vertices.extend(collapse_edge_result.added_vertices.iter().copied());

                let mut neighborhood = collapse_edge_result.added_vertices;
                neighborhood.push(start_vertex_id);

                neighborhood
            };

            halfedges_to_collapse.remove(&min_he_id);

            for removed_he_id in collapse_edge_result.removed_halfedges {
                halfedges_to_collapse.remove(&removed_he_id);
            }

            let mut halfedges_to_check = HashSet::new();

            for vertex_id in vertex_neighborhoods_to_check {
                let Some(outgoing_halfedges) = self.outgoing_halfedges.get(vertex_id) else {
                    // the vertex might have been removed by a previous cleanup
                    continue;
                };

                for &halfedge_id in outgoing_halfedges {
                    let Some(halfedge) = self.halfedges.get(halfedge_id) else {
                        error!("Halfedge not found");
                        continue;
                    };

                    let twin_id = unwrap_or_return!(halfedge.twin, "Twin not found");

                    halfedges_to_check.insert(halfedge_id.min(twin_id));

                    if let Some(face_id) = halfedge.face {
                        if let Some(face) = self.faces.get(face_id) {
                            self.bvh
                                .insert_or_update_partially(face.aabb(self), face.index, 0.0);
                        } else {
                            error!("Face not found. BVH will not be updated.");
                        }
                    }
                }
            }

            for he_id in halfedges_to_check {
                let he = self.halfedges[he_id]; // already checked above

                let len_sqr = he.length_squared(self);

                if len_sqr < min_length_squared {
                    halfedges_to_collapse.insert(he_id, len_sqr);
                } else {
                    halfedges_to_collapse.remove(&he_id);
                }
            }
        }

        #[cfg(feature = "rerun")]
        self.log_rerun();
    }

    #[inline]
    pub fn can_collapse_edge(&mut self, halfedge_id: HalfedgeId) -> bool {
        self.can_collapse_edge_inner(halfedge_id).is_some()
    }

    #[instrument(skip(self))]
    fn can_collapse_edge_inner(
        &mut self,
        halfedge_id: HalfedgeId,
    ) -> Option<(HalfedgeId, VertexId, VertexId, Vec3)> {
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
        let twin_id = he.twin.or_else(error_none!("Twin halfedge not found"))?;
        let twin = self
            .halfedges
            .get(twin_id)
            .or_else(error_none!("Twin halfedge not found"))?;

        let start_vertex_id = twin.end_vertex;
        self.vertices
            .get_mut(start_vertex_id)
            .or_else(error_none!("Start vertex not found"))?
            .outgoing_halfedge = Some(halfedge_id);

        let end_vertex_id = he.end_vertex;
        self.vertices
            .get_mut(end_vertex_id)
            .or_else(error_none!("End vertex not found"))?
            .outgoing_halfedge = Some(twin_id);

        let start_pos = self
            .positions
            .get(start_vertex_id)
            .or_else(error_none!("Start position not found"))?;

        let end_pos = self
            .positions
            .get(end_vertex_id)
            .or_else(error_none!("End position not found"))?;

        let center = (start_pos + end_pos) * 0.5;

        self.check_inverted_faces(start_vertex_id, center)?;
        self.check_inverted_faces(end_vertex_id, center)?;

        Some((twin_id, start_vertex_id, end_vertex_id, center))
    }

    fn check_inverted_faces(&self, vertex_id: VertexId, center: Vec3) -> Option<()> {
        // just made sure that this exists
        let face_ids = self.vertices[vertex_id].faces(self).skip(2).collect_vec();

        for face_id in face_ids {
            let mut orig_positions = Vec::with_capacity(3);
            let mut new_positions = Vec::with_capacity(3);

            let face = self
                .faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?;

            for v_id in face.vertices(self) {
                let pos = *self
                    .positions
                    .get(v_id)
                    .or_else(error_none!("Vertex pos not found"))?;

                if v_id == vertex_id {
                    new_positions.push(center);
                } else {
                    new_positions.push(pos);
                }
                orig_positions.push(pos);
            }

            let orig_normal = Face::normal_from_positions(&orig_positions);
            let new_normal = Face::normal_from_positions(&new_positions);

            if orig_normal.dot(new_normal) < 0.0 {
                return None;
            }
        }

        Some(())
    }

    #[instrument(skip(self))]
    pub fn collapse_edge_inner(
        &mut self,
        halfedge_id: HalfedgeId,
        twin_id: HalfedgeId,
        start_v_id: VertexId,
        end_v_id: VertexId,
        center_pos: Vec3,
    ) -> CollapseEdge {
        let mut result = CollapseEdge::default();

        #[cfg(feature = "rerun")]
        {
            self.log_he_rerun("collapse/he", halfedge_id);
        }
        // TODO : consider border vertices

        let end_outgoing_halfedges = self.vertices[end_v_id]
            .outgoing_halfedges(self)
            .collect::<Vec<_>>();
        let end_incoming_halfedges = self.vertices[end_v_id]
            .incoming_halfedges(self)
            .collect::<Vec<_>>();

        let he = self.halfedges[halfedge_id];
        let twin = self.halfedges[twin_id];

        if !he.is_boundary() {
            let (face_id, halfedge_ids) = unwrap_or_return!(
                self.remove_halfedge_face(halfedge_id),
                "Could not remove face",
                result
            );

            self.remove_outgoing_halfedge(end_v_id, halfedge_ids[0]);

            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(halfedge_ids);
        }
        result.removed_halfedges.push(halfedge_id);

        self.remove_outgoing_halfedge(start_v_id, halfedge_id);

        if !twin.is_boundary() {
            let (face_id, halfedge_ids) = unwrap_or_return!(
                self.remove_halfedge_face(twin_id),
                "Failed to remove halfedge face",
                result
            );

            self.remove_outgoing_halfedge(start_v_id, halfedge_ids[0]);

            result.removed_faces.push(face_id);
            result.removed_halfedges.extend(halfedge_ids);
        }
        result.removed_halfedges.push(twin_id);

        self.remove_outgoing_halfedge(end_v_id, twin_id);

        let outgoing_end_halfedges = self
            .outgoing_halfedges
            .get(end_v_id)
            .cloned()
            .unwrap_or_default();
        self.outgoing_halfedges
            .entry(start_v_id)
            .unwrap() // unwrap is safe here because key exists because we accessed it above in self.positions
            .or_default()
            .extend(outgoing_end_halfedges);

        for end_incoming_he in end_incoming_halfedges {
            if let Some(end_incoming_he_mut) = self.halfedges.get_mut(end_incoming_he) {
                end_incoming_he_mut.end_vertex = start_v_id;
            }
        }

        self.remove_only_vertex(end_v_id);
        result.removed_vertices.push(end_v_id);

        self.halfedges.remove(halfedge_id);
        self.halfedges.remove(twin_id);

        self.positions[start_v_id] = center_pos;

        let new_outgoing_he_id = end_outgoing_halfedges
            .into_iter()
            .find(|he_id| self.halfedges.contains_key(*he_id));

        if let Some(new_outgoing_he_id) = new_outgoing_he_id {
            self.vertices[start_v_id].outgoing_halfedge = Some(new_outgoing_he_id);

            #[cfg(feature = "rerun")]
            {
                self.log_he_rerun(
                    "collapse/outgoing",
                    self.vertices[start_v_id].outgoing_halfedge.unwrap(),
                );
            }

            let cleanup = self.make_vertex_neighborhood_manifold(start_v_id);

            result.added_vertices = cleanup.added_vertices;
            result.removed_vertices.extend(cleanup.removed_vertices);
            result.removed_halfedges.extend(cleanup.removed_halfedges);
            result.removed_faces.extend(cleanup.removed_faces);
        } else {
            self.remove_only_vertex(start_v_id);

            result.removed_vertices.push(start_v_id);
        }

        result
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
    pub fn collapse_edge(&mut self, halfedge_id: HalfedgeId) -> CollapseEdge {
        let he = *unwrap_or_return!(
            self.halfedges.get(halfedge_id),
            "Halfedge not found",
            CollapseEdge::default()
        );
        let twin_id = unwrap_or_return!(he.twin, "Twin missing", CollapseEdge::default());
        let twin = unwrap_or_return!(
            self.halfedges.get(twin_id),
            "Halfedge not found",
            CollapseEdge::default()
        );

        let start_v_id = twin.end_vertex;
        let end_v_id = he.end_vertex;

        let start_pos = *unwrap_or_return!(
            self.positions.get(start_v_id),
            "Start position not found",
            CollapseEdge::default()
        );
        let end_pos = *unwrap_or_return!(
            self.positions.get(end_v_id),
            "End position not found",
            CollapseEdge::default()
        );

        let center_pos = (start_pos + end_pos) * 0.5;

        self.collapse_edge_inner(halfedge_id, twin_id, start_v_id, end_v_id, center_pos)
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

        self.bvh.remove(
            self.faces
                .get(face_id)
                .or_else(error_none!("Face not found"))?
                .index,
        );

        self.halfedges.remove(next_he_id);
        self.halfedges.remove(prev_he_id);
        self.remove_outgoing_halfedge(next_end_v_id, prev_he_id);

        if let Some(face) = self.faces.remove(face_id) {
            self.bvh.remove(face.index);
        }

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

#[derive(Default, Debug)]
pub struct CollapseEdge {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    pub added_vertices: Vec<VertexId>,
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    #[allow(unused_variables)]
    fn test_collapse_edge() {
        let mut mesh_graph = MeshGraph::new();

        let face1 = mesh_graph
            .add_face_from_positions(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 4.0, 0.0),
                Vec3::new(1.0, 2.0, 0.0),
            )
            .face_id;

        let he1 = mesh_graph.faces[face1]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[1];

        let face2 = mesh_graph
            .add_face_from_halfedge_and_position(he1, Vec3::new(2.0, 4.0, 0.0))
            .unwrap()
            .face_id;

        let he2 = mesh_graph.faces[face2]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[2];

        let face3 = mesh_graph
            .add_face_from_halfedge_and_position(he2, Vec3::new(3.0, 2.0, 0.0))
            .unwrap()
            .face_id;

        let he3 = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .collect::<Vec<_>>()[1];

        let face4 = mesh_graph
            .add_face_from_halfedge_and_position(he3, Vec3::new(4.0, 4.0, 0.0))
            .unwrap()
            .face_id;

        let he4 = mesh_graph.faces[face4]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face5 = mesh_graph
            .add_face_from_halfedge_and_position(he4, Vec3::new(4.0, 0.0, 0.0))
            .unwrap()
            .face_id;

        let he5 = mesh_graph.faces[face5]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let face6 = mesh_graph
            .add_face_from_halfedge_and_position(he5, Vec3::new(2.0, 0.0, 0.0))
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
            .add_face_from_halfedges(he6, he3)
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

        mesh_graph.add_face_from_halfedges(he1, he7).unwrap();

        assert_eq!(mesh_graph.vertices.len(), 8);
        assert_eq!(mesh_graph.halfedges.len(), 30);
        assert_eq!(mesh_graph.faces.len(), 8);

        #[cfg(feature = "rerun")]
        mesh_graph.log_rerun();

        let edge_to_collapse = mesh_graph.faces[face3]
            .halfedges(&mesh_graph)
            .nth(2)
            .unwrap();

        let start_v_id = mesh_graph.halfedges[edge_to_collapse]
            .start_vertex(&mesh_graph)
            .unwrap();

        assert_eq!(mesh_graph.outgoing_halfedges[start_v_id].len(), 5);

        let CollapseEdge {
            removed_vertices,
            removed_halfedges,
            removed_faces,
            added_vertices,
        } = mesh_graph.collapse_edge(edge_to_collapse);

        #[cfg(feature = "rerun")]
        {
            mesh_graph.log_rerun();
            crate::RR.flush_blocking().unwrap();
        }

        assert_eq!(removed_vertices.len(), 1);
        assert_eq!(removed_halfedges.len(), 6);
        assert_eq!(removed_faces.len(), 2);

        assert_eq!(mesh_graph.vertices.len(), 7);
        assert_eq!(mesh_graph.halfedges.len(), 24);
        assert_eq!(mesh_graph.faces.len(), 6);

        assert_eq!(mesh_graph.outgoing_halfedges[start_v_id].len(), 6);
    }

    #[cfg(feature = "gltf")]
    #[test]
    fn test_can_collapse_edge() {
        use crate::{integrations::gltf, utils::get_tracing_subscriber};

        get_tracing_subscriber();
        let mut meshgraph = gltf::load("src/ops/glb/can_collapse_edge.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let mut v_top_id = VertexId::default();
        let mut v_bottom_id = VertexId::default();

        for (v_id, pos) in &meshgraph.positions {
            if pos.x == 0.0 {
                if pos.y == -1.0 {
                    v_top_id = v_id;
                } else if pos.y == 1.0 {
                    v_bottom_id = v_id;
                }
            }
        }

        if v_top_id == VertexId::default() {
            panic!("No top vertex found");
        }

        if v_bottom_id == VertexId::default() {
            panic!("No bottom vertex found");
        }

        let he_id = meshgraph.halfedge_from_to(v_top_id, v_bottom_id).unwrap();

        let result = meshgraph.can_collapse_edge(he_id);

        assert!(result);

        #[cfg(feature = "rerun")]
        crate::RR.flush_blocking().unwrap();
    }

    #[cfg(feature = "gltf")]
    #[test]
    fn test_cannot_collapse_edge() {
        use crate::{integrations::gltf, utils::get_tracing_subscriber};

        get_tracing_subscriber();
        let mut meshgraph = gltf::load("src/ops/glb/cannot_collapse_edge.glb").unwrap();

        #[cfg(feature = "rerun")]
        meshgraph.log_rerun();

        let mut v_top_id = VertexId::default();
        let mut v_bottom_id = VertexId::default();

        for (v_id, pos) in &meshgraph.positions {
            if pos.x == 0.0 {
                if pos.y == -1.0 {
                    v_top_id = v_id;
                } else if pos.y == 1.0 {
                    v_bottom_id = v_id;
                }
            }
        }

        if v_top_id == VertexId::default() {
            panic!("No top vertex found");
        }

        if v_bottom_id == VertexId::default() {
            panic!("No bottom vertex found");
        }

        let he_id = meshgraph.halfedge_from_to(v_top_id, v_bottom_id).unwrap();

        let result = meshgraph.can_collapse_edge(he_id);

        assert!(!result);

        #[cfg(feature = "rerun")]
        crate::RR.flush_blocking().unwrap();
    }
}
