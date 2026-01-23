use glam::Vec3;
use tracing::{error, instrument};

use crate::{Face, FaceId, Halfedge, HalfedgeId, MeshGraph, Vertex, VertexId, error_none};

impl MeshGraph {
    /// Inserts a vertex and it's position into the mesh graph.
    /// It doesn't do any connections.
    pub fn insert_vertex(&mut self, position: Vec3) -> VertexId {
        let vertex = Vertex::default();
        let vertex_id = self.vertices.insert(vertex);
        self.positions.insert(vertex_id, position);

        self.outgoing_halfedges.insert(vertex_id, vec![]);

        vertex_id
    }

    /// Inserts a pair of halfedges and connects them to the given vertices (from and to the halfedge) and each other as twins.
    /// If the edge already exists or partially exists,
    /// it returns the existing edge while creating any missing halfedges.
    #[instrument(skip(self))]
    pub fn insert_or_get_edge(
        &mut self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> InsertOrGetEdge {
        let ret = self
            .insert_or_get_edge_inner(start_vertex_id, end_vertex_id)
            .unwrap_or_else(|| {
                let start_to_end_he_id = self.insert_halfedge(start_vertex_id, end_vertex_id);
                let twin_he_id = self.insert_halfedge(end_vertex_id, start_vertex_id);

                InsertOrGetEdge {
                    start_to_end_he_id,
                    twin_he_id,
                    new_start_to_end: true,
                    new_twin: true,
                }
            });

        // insert_or_get_edge_inner ensures that both halfedges exist.
        self.halfedges[ret.start_to_end_he_id].twin = Some(ret.twin_he_id);
        self.halfedges[ret.twin_he_id].twin = Some(ret.start_to_end_he_id);

        if let Some(start_v) = self.vertices.get_mut(start_vertex_id) {
            start_v.outgoing_halfedge = Some(ret.start_to_end_he_id);
        } else {
            error!("Vertex not found");
        }

        if let Some(end_v) = self.vertices.get_mut(end_vertex_id) {
            end_v.outgoing_halfedge = Some(ret.twin_he_id);
        } else {
            error!("Vertex not found");
        }

        ret
    }

    /// Returns the halfedge from the start vertex to the end vertex, if it exists. `None` otherwise.
    pub fn halfedge_from_to(
        &mut self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Option<HalfedgeId> {
        self.outgoing_halfedges
            .entry(start_vertex_id)
            .or_else(error_none!("Start vertex not found"))?
            .or_insert_with(Vec::new)
            .iter()
            .copied()
            .find(|he_id| {
                self.halfedges
                    .get(*he_id)
                    .or_else(error_none!("Halfedge not found"))
                    .is_some_and(|he| he.end_vertex == end_vertex_id)
            })
    }

    fn insert_or_get_edge_inner(
        &mut self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Option<InsertOrGetEdge> {
        let mut he1_id = self.halfedge_from_to(start_vertex_id, end_vertex_id);
        let mut he2_id = self.halfedge_from_to(end_vertex_id, start_vertex_id);
        let mut new1 = false;
        let mut new2 = false;

        match (he1_id, he2_id) {
            (Some(h1_id), Some(h2_id)) => {
                let h1 = self
                    .halfedges
                    .get(h1_id)
                    .or_else(error_none!("Halfedge not found"))?;

                let h2 = self
                    .halfedges
                    .get(h2_id)
                    .or_else(error_none!("Halfedge not found"))?;

                if h1.twin != Some(h2_id) || h2.twin != Some(h1_id) {
                    error!("Halfedge twins are not consistent.");
                }
            }
            (Some(_h1_id), None) => {
                he2_id = Some(self.insert_halfedge(end_vertex_id, start_vertex_id));
                new2 = true;
            }
            (None, Some(_h2_id)) => {
                he1_id = Some(self.insert_halfedge(start_vertex_id, end_vertex_id));
                new1 = true;
            }
            (None, None) => {
                // the outer method will handle this case
                return None;
            }
        }

        // we just made sure that h1_id and h2_id are Some(_)
        let start_to_end_he_id = he1_id.unwrap();
        let twin_he_id = he2_id.unwrap();

        Some(InsertOrGetEdge {
            start_to_end_he_id,
            twin_he_id,
            new_start_to_end: new1,
            new_twin: new2,
        })
    }

    /// Inserts a halfedge into the mesh graph. It only connects the halfedge to the given end vertex but not the reverse.
    /// It also doesn't do any other connections.
    /// It inserts into `self.outgoing_halfedges`.
    ///
    /// Use [`insert_or_get_edge`] instead of this when you can to lower the chance of creating an invalid graph.
    pub fn insert_halfedge(&mut self, start_vertex: VertexId, end_vertex: VertexId) -> HalfedgeId {
        let halfedge = Halfedge {
            end_vertex,
            next: None,
            twin: None,
            face: None,
        };
        let he_id = self.halfedges.insert(halfedge);

        self.outgoing_halfedges
            .entry(start_vertex)
            .expect("we just insterted into halfedges above")
            .or_default()
            .push(he_id);

        he_id
    }

    /// Inserts a face into the mesh graph. It connects the halfedges to the face and the face to the first halfedge.
    /// Additionally it connects the halfedges' `next` loop around the face.
    #[instrument(skip(self))]
    pub fn insert_face(
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

        face_id
    }
}

/// Return value of `insert_or_get_edge`
pub struct InsertOrGetEdge {
    /// Id of the halfedge from start vertex to end vertex
    pub start_to_end_he_id: HalfedgeId,
    /// Id of the halfedge from end vertex to start vertex
    pub twin_he_id: HalfedgeId,
    /// Whether the halfedge from start vertex to end vertex was newly created
    pub new_start_to_end: bool,
    /// Whether the halfedge from end vertex to start vertex was newly created
    pub new_twin: bool,
}

impl InsertOrGetEdge {
    /// Return the ids of the newly created halfedges
    pub fn created_he_ids(&self) -> Vec<HalfedgeId> {
        let mut he_ids = vec![];
        if self.new_start_to_end {
            he_ids.push(self.start_to_end_he_id);
        }
        if self.new_twin {
            he_ids.push(self.twin_he_id);
        }
        he_ids
    }
}
