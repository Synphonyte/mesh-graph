use tracing::{error, instrument};

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId, error_none, utils::unwrap_or_return};

impl MeshGraph {
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

    /// Returns the halfedge from the start vertex to the end vertex, if it exists. `None` otherwise.
    #[instrument(skip(self))]
    pub fn halfedge_from_to(
        &self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Option<HalfedgeId> {
        let out_hes = self
            .outgoing_halfedges
            .get(start_vertex_id)
            .or_else(error_none!("Start vertex not found"))?;

        out_hes.iter().copied().find(|he_id| {
            self.halfedges
                .get(*he_id)
                .is_some_and(|he| he.end_vertex == end_vertex_id)
        })
    }

    /// Returns all the halfedges from the start vertex to the end vertex.
    /// In a valid manifold mesh, this should return exactly one halfedge.
    #[instrument(skip(self))]
    pub fn halfedges_from_to(
        &self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Vec<HalfedgeId> {
        let out_hes = unwrap_or_return!(
            self.outgoing_halfedges.get(start_vertex_id),
            "Start vertex not found",
            vec![]
        );

        out_hes
            .iter()
            .copied()
            .filter(|he_id| {
                self.halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.end_vertex == end_vertex_id)
            })
            .collect()
    }

    /// Returns the face with the given vertices, if it exists. `None` otherwise.
    pub fn face_with_vertices(
        &self,
        v_id1: VertexId,
        v_id2: VertexId,
        v_id3: VertexId,
    ) -> Option<FaceId> {
        let he_id = self.halfedge_from_to(v_id1, v_id2)?;
        let he = self
            .halfedges
            .get(he_id)
            .or_else(error_none!("Halfedge not found"))?;

        if Some(v_id3) == he.opposite_vertex(self) {
            return he.face.or_else(error_none!("No face"));
        }

        let twin_id = he.twin.or_else(error_none!("No twin"))?;
        let twin = self
            .halfedges
            .get(twin_id)
            .or_else(error_none!("Twin not found"))?;

        if Some(v_id3) == twin.opposite_vertex(self) {
            return twin.face.or_else(error_none!("No face"));
        }

        None
    }

    /// Returns an iterator of all adjacent faces to the give vertex.
    ///
    /// In contrast to `Vertex.faces()` it doesn't guarantee a specific order. But it
    /// uses `MeshGraph::outgoing_halfedges` so it can continue to work with non-manifold
    /// mesh graphs while `Vertex.faces()` only works with manifold ones.
    #[instrument(skip(self))]
    pub fn vertex_adjacent_faces(&self, vertex_id: VertexId) -> Vec<FaceId> {
        let Some(he_ids) = self.outgoing_halfedges.get(vertex_id) else {
            error!("No outgoing halfedges entry for vertex {vertex_id:?}");
            return vec![];
        };

        he_ids
            .iter()
            .filter_map(|he_id| {
                self.halfedges
                    .get(*he_id)
                    .or_else(error_none!("Halfedge not found for {he_id:?}"))
                    .and_then(|he| he.face)
            })
            .collect()
    }
}
