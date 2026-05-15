use glam::Vec3;
use tracing::{error, instrument};

use crate::{Face, FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

impl MeshGraph {
    #[instrument(skip(self))]
    pub fn add_face_from_positions(&mut self, a: Vec3, b: Vec3, c: Vec3) -> AddFace {
        let a_id = self.add_vertex(a);
        let b_id = self.add_vertex(b);
        let c_id = self.add_vertex(c);

        let inserted_a = self.add_or_get_edge(a_id, b_id);
        let inserted_b = self.add_or_get_edge(b_id, c_id);
        let inserted_c = self.add_or_get_edge(c_id, a_id);

        let face_id = self.add_face(
            inserted_a.start_to_end_he_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_a.created_he_ids();
        halfedge_ids.extend(inserted_b.created_he_ids());
        halfedge_ids.extend(inserted_c.created_he_ids());

        AddFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![a_id, b_id, c_id],
        }
    }

    /// Returns `None` when an edge already has two faces
    #[instrument(skip(self))]
    pub fn add_face_from_halfedge_and_position(
        &mut self,
        he_id: HalfedgeId,
        opposite_vertex_pos: Vec3,
    ) -> Option<AddFace> {
        let vertex_id = self.add_vertex(opposite_vertex_pos);
        self.add_face_from_halfedge_and_vertex(he_id, vertex_id)
    }

    /// Returns `None` when the edge described by `he_id` already has two faces
    #[instrument(skip(self))]
    pub fn add_face_from_halfedge_and_vertex(
        &mut self,
        he_id: HalfedgeId,
        vertex_id: VertexId,
    ) -> Option<AddFace> {
        let he_a_id = self.boundary_he(he_id)?;

        let he_a = self.halfedges[he_a_id];
        let start_vertex_id_he_a = he_a.start_vertex(self)?;
        let end_vertex_id_he_a = he_a.end_vertex;

        let inserted_b = self.add_or_get_edge(end_vertex_id_he_a, vertex_id);
        let inserted_c = self.add_or_get_edge(vertex_id, start_vertex_id_he_a);

        let face_id = self.add_face(
            he_a_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_b.created_he_ids();
        halfedge_ids.extend(inserted_c.created_he_ids());

        Some(AddFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![],
        })
    }

    /// Creates a face from three vertices.
    #[instrument(skip(self))]
    pub fn add_face_from_vertices(
        &mut self,
        v_id1: VertexId,
        v_id2: VertexId,
        v_id3: VertexId,
    ) -> AddFace {
        let inserted_a = self.add_or_get_edge(v_id1, v_id2);
        let inserted_b = self.add_or_get_edge(v_id2, v_id3);
        let inserted_c = self.add_or_get_edge(v_id3, v_id1);

        let face_id = self.add_face(
            inserted_a.start_to_end_he_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_a.created_he_ids();
        halfedge_ids.extend(inserted_b.created_he_ids());
        halfedge_ids.extend(inserted_c.created_he_ids());

        AddFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![],
        }
    }

    /// Creates a face from two halfedges.
    #[instrument(skip(self))]
    pub fn add_face_from_halfedges(
        &mut self,
        he_id1: HalfedgeId,
        he_id2: HalfedgeId,
    ) -> Option<AddFace> {
        let he_id1 = self
            .boundary_he(he_id1)
            .or_else(error_none!("Boundary Halfedge 1 {he_id1:?} not found"))?;
        let he_id2 = self
            .boundary_he(he_id2)
            .or_else(error_none!("Boundary Halfedge 2 {he_id2:?} not found"))?;

        let he1 = *self
            .halfedges
            .get(he_id1)
            .or_else(error_none!("Halfedge 1 not found"))?;
        let he2 = *self
            .halfedges
            .get(he_id2)
            .or_else(error_none!("Halfedge 2 not found"))?;

        let he1_start_vertex = he1
            .start_vertex(self)
            .or_else(error_none!("Start vertex should be available"))?;

        let he2_start_vertex = he2
            .start_vertex(self)
            .or_else(error_none!("Start vertex should be available"))?;

        if he1_start_vertex == he2.end_vertex {
            let inserted = self.add_or_get_edge(he1.end_vertex, he2_start_vertex);

            let face_id = self.add_face(inserted.start_to_end_he_id, he_id2, he_id1);

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else if he1_start_vertex == he2_start_vertex {
            let inserted = self.add_or_get_edge(he1.end_vertex, he2.end_vertex);

            let face_id = self.add_face(
                inserted.start_to_end_he_id,
                he2.twin.or_else(error_none!("Twin not set"))?,
                he_id1,
            );

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else if he1.end_vertex == he2.end_vertex {
            let inserted = self.add_or_get_edge(he2_start_vertex, he1_start_vertex);

            let face_id = self.add_face(
                inserted.start_to_end_he_id,
                he_id1,
                he2.twin.or_else(error_none!("Twin not set"))?,
            );

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else {
            let inserted = self.add_or_get_edge(he2.end_vertex, he1_start_vertex);

            let face_id = self.add_face(inserted.start_to_end_he_id, he_id1, he_id2);

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        }
    }

    /// Inserts a face into the mesh graph. It connects the halfedges to the face and the face to the first halfedge.
    /// Additionally it connects the halfedges' `next` loop around the face.
    #[instrument(skip(self))]
    pub fn add_face(
        &mut self,
        he1_id: HalfedgeId,
        he2_id: HalfedgeId,
        he3_id: HalfedgeId,
    ) -> FaceId {
        let face_id = self.faces.insert_with_key(|id| Face {
            halfedge: he1_id,
            index: self.next_index,
            id,
        });

        self.index_to_face_id.insert(self.next_index, face_id);

        self.next_index += 1;

        for (he_id, next_he_id) in [(he1_id, he2_id), (he2_id, he3_id), (he3_id, he1_id)] {
            if let Some(halfedge) = self.halfedges.get_mut(he_id) {
                halfedge.face = Some(face_id);
                halfedge.next = Some(next_he_id);
            } else {
                error!("Halfedge not found");
            }
        }

        let face = self.faces[face_id]; // just inserted above
        self.bvh
            .insert_or_update_partially(face.aabb(self), face.index, 0.0);

        face_id
    }
}

/// Return value of several `add_face...` methods
pub struct AddFace {
    /// Id of the created face
    pub face_id: FaceId,
    /// During the process of creating the face all new created halfedges
    pub halfedge_ids: Vec<HalfedgeId>,
    /// During the process of creating the face all new created vertices
    pub vertex_ids: Vec<VertexId>,
}

#[cfg(test)]
mod test {
    use super::*;

    fn add_face(mesh_graph: &mut MeshGraph) -> FaceId {
        mesh_graph
            .add_face_from_positions(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .face_id
    }

    fn add_face_to_edge(
        mesh_graph: &mut MeshGraph,
        face_id: FaceId,
        pos: Vec3,
        use_twin: bool,
    ) -> Option<FaceId> {
        let mut associated_he_id = mesh_graph.faces[face_id]
            .halfedges(mesh_graph)
            .nth(1)
            .unwrap();

        if use_twin {
            associated_he_id = mesh_graph.halfedges[associated_he_id].twin.unwrap();
        }

        mesh_graph
            .add_face_from_halfedge_and_position(associated_he_id, pos)
            .map(|f| f.face_id)
    }

    fn fill_face(
        mesh_graph: &mut MeshGraph,
        face_id_1: FaceId,
        face_id_2: FaceId,
        use_twin: bool,
    ) -> Option<FaceId> {
        let mut he_id_1 = mesh_graph.faces[face_id_1]
            .halfedges(mesh_graph)
            .next()
            .unwrap();

        let mut he_id_2 = mesh_graph.faces[face_id_2]
            .halfedges(mesh_graph)
            .nth(1)
            .unwrap();

        if use_twin {
            he_id_1 = mesh_graph.halfedges[he_id_1].twin.unwrap();
            he_id_2 = mesh_graph.halfedges[he_id_2].twin.unwrap();
        }

        mesh_graph
            .add_face_from_halfedges(he_id_1, he_id_2)
            .map(|f| f.face_id)
    }

    macro_rules! init_fill_face {
        ($f1:ident, $f2:ident, $f3:ident, $mg:ident) => {
            let $f1 = add_face(&mut $mg);

            let $f2 = add_face_to_edge(&mut $mg, $f1, Vec3::new(1.0, 1.0, 0.0), false);
            let $f3 = add_face_to_edge(&mut $mg, $f2.unwrap(), Vec3::new(0.5, 0.5, 1.0), false);
        };
    }

    macro_rules! log_faces_rerun {
        ($mg:ident, $($face:expr),*) => {
            #[cfg(feature = "rerun")]
            {
                $(
                    $mg.log_face_rerun(&format!("{:?}", $face), $face);
                )*
            }
        };
    }

    fn test_vertices(mesh_graph: &MeshGraph, face: FaceId) {
        let mut vertices = mesh_graph.faces[face].vertices(mesh_graph);

        let vertex = vertices.next();
        assert!(vertex.is_some());
        assert!(
            mesh_graph.vertices[vertex.unwrap()]
                .outgoing_halfedge
                .is_some()
        );

        let vertex = vertices.next();
        assert!(vertex.is_some());
        assert!(
            mesh_graph.vertices[vertex.unwrap()]
                .outgoing_halfedge
                .is_some()
        );

        let vertex = vertices.next();
        assert!(vertex.is_some());
        assert!(
            mesh_graph.vertices[vertex.unwrap()]
                .outgoing_halfedge
                .is_some()
        );

        assert!(vertices.next().is_none());
    }

    fn test_halfedges(mesh_graph: &MeshGraph, face: FaceId) {
        let mut halfedges = mesh_graph.faces[face].halfedges(mesh_graph);

        assert!(halfedges.next().is_some());
        assert!(halfedges.next().is_some());
        assert!(halfedges.next().is_some());
        assert!(halfedges.next().is_none());
    }

    #[test]
    fn test_create_face() {
        let mut mesh_graph = MeshGraph::new();
        let face1 = add_face(&mut mesh_graph);

        assert_eq!(mesh_graph.vertices.len(), 3);
        assert_eq!(mesh_graph.positions.len(), 3);
        assert_eq!(mesh_graph.halfedges.len(), 6);
        assert_eq!(mesh_graph.faces.len(), 1);

        test_vertices(&mesh_graph, face1);
        test_halfedges(&mesh_graph, face1);

        log_faces_rerun!(mesh_graph, face1);
    }

    #[test]
    fn test_add_face_to_edge() {
        let mut mesh_graph = MeshGraph::new();
        let face1 = add_face(&mut mesh_graph);

        let face2 = add_face_to_edge(&mut mesh_graph, face1, Vec3::new(1.0, 1.0, 0.0), false);

        assert!(face2.is_some());
        assert_eq!(mesh_graph.vertices.len(), 4);
        assert_eq!(mesh_graph.positions.len(), 4);
        assert_eq!(mesh_graph.halfedges.len(), 10);
        assert_eq!(mesh_graph.faces.len(), 2);

        test_vertices(&mesh_graph, face2.unwrap());
        test_halfedges(&mesh_graph, face2.unwrap());

        log_faces_rerun!(mesh_graph, face1, face2.unwrap());
    }

    #[test]
    fn test_add_face_to_twin_edge() {
        let mut mesh_graph = MeshGraph::new();
        let face1 = add_face(&mut mesh_graph);

        let face2 = add_face_to_edge(&mut mesh_graph, face1, Vec3::new(1.0, 1.0, 0.0), true);

        assert!(face2.is_some());
        assert_eq!(mesh_graph.vertices.len(), 4);
        assert_eq!(mesh_graph.positions.len(), 4);
        assert_eq!(mesh_graph.halfedges.len(), 10);
        assert_eq!(mesh_graph.faces.len(), 2);

        test_vertices(&mesh_graph, face2.unwrap());
        test_halfedges(&mesh_graph, face2.unwrap());

        log_faces_rerun!(mesh_graph, face1, face2.unwrap());
    }

    #[test]
    fn test_fill_face() {
        let mut mesh_graph = MeshGraph::new();
        init_fill_face!(face1, face2, face3, mesh_graph);

        let face4 = fill_face(&mut mesh_graph, face1, face3.unwrap(), false);

        assert!(face4.is_some());
        assert_eq!(mesh_graph.vertices.len(), 5);
        assert_eq!(mesh_graph.positions.len(), 5);
        assert_eq!(mesh_graph.halfedges.len(), 16);
        assert_eq!(mesh_graph.faces.len(), 4);

        test_vertices(&mesh_graph, face4.unwrap());
        test_halfedges(&mesh_graph, face4.unwrap());

        log_faces_rerun!(
            mesh_graph,
            face1,
            face2.unwrap(),
            face3.unwrap(),
            face4.unwrap()
        );
    }

    #[test]
    fn test_fill_face_from_twin_edges() {
        let mut mesh_graph = MeshGraph::new();
        init_fill_face!(face1, face2, face3, mesh_graph);

        let face4 = fill_face(&mut mesh_graph, face1, face3.unwrap(), true);

        assert!(face4.is_some());
        assert_eq!(mesh_graph.vertices.len(), 5);
        assert_eq!(mesh_graph.positions.len(), 5);
        assert_eq!(mesh_graph.halfedges.len(), 16);
        assert_eq!(mesh_graph.faces.len(), 4);

        test_vertices(&mesh_graph, face4.unwrap());
        test_halfedges(&mesh_graph, face4.unwrap());

        log_faces_rerun!(
            mesh_graph,
            face1,
            face2.unwrap(),
            face3.unwrap(),
            face4.unwrap()
        );
    }
}
