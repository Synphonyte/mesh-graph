use glam::Vec3;
use tracing::instrument;

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

impl MeshGraph {
    pub fn create_face_from_positions(&mut self, a: Vec3, b: Vec3, c: Vec3) -> FaceId {
        let a_id = self.insert_vertex(a);
        let b_id = self.insert_vertex(b);
        let c_id = self.insert_vertex(c);

        let (he_a_id, _) = self.insert_or_get_edge(a_id, b_id);
        let (he_b_id, _) = self.insert_or_get_edge(b_id, c_id);
        let (he_c_id, _) = self.insert_or_get_edge(c_id, a_id);

        let face_id = self.insert_face(he_a_id, he_b_id, he_c_id);

        face_id
    }

    /// Returns `None` when an edge already has two faces
    #[instrument(skip(self))]
    pub fn create_face_from_halfedge_and_position(
        &mut self,
        he_id: HalfedgeId,
        opposite_vertex_pos: Vec3,
    ) -> Option<FaceId> {
        let vertex_id = self.insert_vertex(opposite_vertex_pos);
        self.create_face_from_halfedge_and_vertex(he_id, vertex_id)
    }

    /// Returns `None` when an edge already has two faces
    #[instrument(skip(self))]
    pub fn create_face_from_halfedge_and_vertex(
        &mut self,
        he_id: HalfedgeId,
        vertex_id: VertexId,
    ) -> Option<FaceId> {
        let he_a_id = self.boundary_he(he_id)?;

        let he_a = self.halfedges[he_a_id];
        let start_vertex_id_he_a = he_a.start_vertex(self)?;
        let end_vertex_id_he_a = he_a.end_vertex;

        let (he_b_id, _) = self.insert_or_get_edge(end_vertex_id_he_a, vertex_id);
        let (he_c_id, _) = self.insert_or_get_edge(vertex_id, start_vertex_id_he_a);

        Some(self.insert_face(he_a_id, he_b_id, he_c_id))
    }

    #[instrument(skip(self))]
    pub fn create_face_from_halfedges(
        &mut self,
        he_id1: HalfedgeId,
        he_id2: HalfedgeId,
    ) -> Option<FaceId> {
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
            let (he_id3, _) = self.insert_or_get_edge(he1.end_vertex, he2_start_vertex);

            Some(self.insert_face(he_id3, he_id2, he_id1))
        } else if he1_start_vertex == he2_start_vertex {
            let (he_id3, _) = self.insert_or_get_edge(he1.end_vertex, he2.end_vertex);

            Some(self.insert_face(
                he_id3,
                he2.twin.or_else(error_none!("Twin not set"))?,
                he_id1,
            ))
        } else if he1.end_vertex == he2.end_vertex {
            let (he_id3, _) = self.insert_or_get_edge(he2_start_vertex, he1_start_vertex);

            Some(self.insert_face(
                he_id3,
                he_id1,
                he2.twin.or_else(error_none!("Twin not set"))?,
            ))
        } else {
            let (he_id3, _) = self.insert_or_get_edge(he2.end_vertex, he1_start_vertex);

            Some(self.insert_face(he_id3, he_id1, he_id2))
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
}

#[cfg(test)]
mod test {
    use super::*;

    fn create_face(mesh_graph: &mut MeshGraph) -> FaceId {
        mesh_graph.create_face_from_positions(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }

    fn add_face_to_edge(
        mesh_graph: &mut MeshGraph,
        face_id: FaceId,
        pos: Vec3,
        use_twin: bool,
    ) -> Option<FaceId> {
        let mut associated_he_id = mesh_graph.faces[face_id]
            .halfedges(&mesh_graph)
            .into_iter()
            .collect::<Vec<_>>()[1];

        if use_twin {
            associated_he_id = mesh_graph.halfedges[associated_he_id].twin.unwrap();
        }

        mesh_graph.create_face_from_halfedge_and_position(associated_he_id, pos)
    }

    fn fill_face(
        mesh_graph: &mut MeshGraph,
        face_id_1: FaceId,
        face_id_2: FaceId,
        use_twin: bool,
    ) -> Option<FaceId> {
        let mut he_id_1 = mesh_graph.faces[face_id_1]
            .halfedges(&mesh_graph)
            .into_iter()
            .collect::<Vec<_>>()[0];

        let mut he_id_2 = mesh_graph.faces[face_id_2]
            .halfedges(&mesh_graph)
            .into_iter()
            .collect::<Vec<_>>()[1];

        if use_twin {
            he_id_1 = mesh_graph.halfedges[he_id_1].twin.unwrap();
            he_id_2 = mesh_graph.halfedges[he_id_2].twin.unwrap();
        }

        mesh_graph.create_face_from_halfedges(he_id_1, he_id_2)
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
