use glam::Vec3;
use tracing::instrument;

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none, utils::unwrap_or_return};

impl MeshGraph {
    #[instrument(skip(self))]
    pub fn create_face_from_positions(&mut self, a: Vec3, b: Vec3, c: Vec3) -> CreateFace {
        let a_id = self.insert_vertex(a);
        let b_id = self.insert_vertex(b);
        let c_id = self.insert_vertex(c);

        let inserted_a = self.insert_or_get_edge(a_id, b_id);
        let inserted_b = self.insert_or_get_edge(b_id, c_id);
        let inserted_c = self.insert_or_get_edge(c_id, a_id);

        let face_id = self.insert_face(
            inserted_a.start_to_end_he_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_a.created_he_ids();
        halfedge_ids.extend(inserted_b.created_he_ids());
        halfedge_ids.extend(inserted_c.created_he_ids());

        CreateFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![a_id, b_id, c_id],
        }
    }

    /// Returns `None` when an edge already has two faces
    #[instrument(skip(self))]
    pub fn create_face_from_halfedge_and_position(
        &mut self,
        he_id: HalfedgeId,
        opposite_vertex_pos: Vec3,
    ) -> Option<CreateFace> {
        let vertex_id = self.insert_vertex(opposite_vertex_pos);
        self.create_face_from_halfedge_and_vertex(he_id, vertex_id)
    }

    /// Returns `None` when the edge described by `he_id` already has two faces
    #[instrument(skip(self))]
    pub fn create_face_from_halfedge_and_vertex(
        &mut self,
        he_id: HalfedgeId,
        vertex_id: VertexId,
    ) -> Option<CreateFace> {
        let he_a_id = self.boundary_he(he_id)?;

        let he_a = self.halfedges[he_a_id];
        let start_vertex_id_he_a = he_a.start_vertex(self)?;
        let end_vertex_id_he_a = he_a.end_vertex;

        let inserted_b = self.insert_or_get_edge(end_vertex_id_he_a, vertex_id);
        let inserted_c = self.insert_or_get_edge(vertex_id, start_vertex_id_he_a);

        let face_id = self.insert_face(
            he_a_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_b.created_he_ids();
        halfedge_ids.extend(inserted_c.created_he_ids());

        Some(CreateFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![],
        })
    }

    /// Creates a face from three vertices.
    #[instrument(skip(self))]
    pub fn create_face_from_vertices(
        &mut self,
        v_id1: VertexId,
        v_id2: VertexId,
        v_id3: VertexId,
    ) -> Option<CreateFace> {
        let inserted_a = self.insert_or_get_edge(v_id1, v_id2);
        let inserted_b = self.insert_or_get_edge(v_id2, v_id3);
        let inserted_c = self.insert_or_get_edge(v_id3, v_id1);

        let face_id = self.insert_face(
            inserted_a.start_to_end_he_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_a.created_he_ids();
        halfedge_ids.extend(inserted_b.created_he_ids());
        halfedge_ids.extend(inserted_c.created_he_ids());

        Some(CreateFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![],
        })
    }

    /// Creates a face from two halfedges.
    #[instrument(skip(self))]
    pub fn create_face_from_halfedges(
        &mut self,
        he_id1: HalfedgeId,
        he_id2: HalfedgeId,
    ) -> Option<CreateFace> {
        let he_id1 = self.boundary_he(he_id1)?;
        let he_id2 = self.boundary_he(he_id2)?;

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
            let inserted = self.insert_or_get_edge(he1.end_vertex, he2_start_vertex);

            let face_id = self.insert_face(inserted.start_to_end_he_id, he_id2, he_id1);

            Some(CreateFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else if he1_start_vertex == he2_start_vertex {
            let inserted = self.insert_or_get_edge(he1.end_vertex, he2.end_vertex);

            let face_id = self.insert_face(
                inserted.start_to_end_he_id,
                he2.twin.or_else(error_none!("Twin not set"))?,
                he_id1,
            );

            Some(CreateFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else if he1.end_vertex == he2.end_vertex {
            let inserted = self.insert_or_get_edge(he2_start_vertex, he1_start_vertex);

            let face_id = self.insert_face(
                inserted.start_to_end_he_id,
                he_id1,
                he2.twin.or_else(error_none!("Twin not set"))?,
            );

            Some(CreateFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else {
            let inserted = self.insert_or_get_edge(he2.end_vertex, he1_start_vertex);

            let face_id = self.insert_face(inserted.start_to_end_he_id, he_id1, he_id2);

            Some(CreateFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        }
    }

    /// Return the halfedge or it's twin depending on which one is boundary, or `None` if both are not boundary.
    #[instrument(skip(self))]
    pub fn boundary_he(&self, he_id: HalfedgeId) -> Option<HalfedgeId> {
        let he = *self
            .halfedges
            .get(he_id)
            .or_else(error_none!("Halfedge not found"))?;

        if he.is_boundary() {
            return Some(he_id);
        }

        let twin_id = he.twin.or_else(error_none!("Twin missing"))?;

        let twin_he = self
            .halfedges
            .get(twin_id)
            .or_else(error_none!("Twin not found"))?;

        if twin_he.is_boundary() {
            return Some(twin_id);
        }

        None
    }

    /// Returns the vertex order of the boundary halfedge between two vertices.
    /// Useful when adding faces on boundaries.
    ///
    /// If the edge is not boundary, it always returns `[v_id2, v_id1]`.
    #[instrument(skip(self))]
    pub fn boundary_vertex_order(&mut self, v_id1: VertexId, v_id2: VertexId) -> [VertexId; 2] {
        let he_id = unwrap_or_return!(
            self.halfedge_from_to(v_id1, v_id2),
            "Couldn't find halfedge between {v_id1:?} and {v_id2:?}",
            [v_id1, v_id2]
        );

        // already checked that the halfedge exists in `halfedge_from_to()`
        let he = self.halfedges[he_id];

        if he.is_boundary() {
            [v_id1, v_id2]
        } else {
            [v_id2, v_id1]
        }
    }
}

/// Return value of several `create_face...` methods
pub struct CreateFace {
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

    fn create_face(mesh_graph: &mut MeshGraph) -> FaceId {
        mesh_graph
            .create_face_from_positions(
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
            .create_face_from_halfedge_and_position(associated_he_id, pos)
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
            .create_face_from_halfedges(he_id_1, he_id_2)
            .map(|f| f.face_id)
    }

    macro_rules! init_fill_face {
        ($f1:ident, $f2:ident, $f3:ident, $mg:ident) => {
            let $f1 = create_face(&mut $mg);

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
                crate::RR.flush_blocking().unwrap();
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
        let face1 = create_face(&mut mesh_graph);

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
        let face1 = create_face(&mut mesh_graph);

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
        let face1 = create_face(&mut mesh_graph);

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
