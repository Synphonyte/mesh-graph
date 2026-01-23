use tracing::instrument;

use crate::{HalfedgeId, MeshGraph, VertexId, utils::unwrap_or_return};

mod cleanup;
mod collapse;
mod create;
mod delete;
mod insert;
mod query;
mod subdivide;

impl MeshGraph {
    /// Flips this edge so that it represents the other diagonal described by the quad formed by the two incident triangles.
    ///
    /// ```text
    ///      .                    .
    ///     ( )                  ( )
    ///     ╱'╲                  ╱'╲
    ///    ╱   ╲                ╱ ║ ╲
    ///  .╱     ╲.            .╱  ║  ╲.
    /// ( )═════( )    =>    ( )  ║  ( )
    ///  '╲     ╱'            '╲  ║  ╱'
    ///    ╲   ╱                ╲ ║ ╱
    ///     ╲.╱                  ╲.╱
    ///     ( )                  ( )
    ///      '                    '
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

        self.change_outgoing_halfedge(start_v_id, prev_he_id);
        self.change_outgoing_halfedge(opposite_v_id, next_he_id);

        self.halfedges[twin_he_id].end_vertex = twin_opposite_v_id;

        self.halfedges[twin_prev_he_id].end_vertex = opposite_v_id;
        self.make_twins(twin_prev_he_id, next_twin_he_id);
        self.halfedges[twin_next_he_id].end_vertex = twin_start_v_id;
        self.make_twins(twin_next_he_id, twin_prev_twin_he_id);

        self.change_outgoing_halfedge(twin_start_v_id, twin_prev_he_id);
        self.change_outgoing_halfedge(twin_opposite_v_id, twin_next_he_id);
    }

    /// Makes two halfedges twins of each other. Doesn't change anything else
    pub fn make_twins(&mut self, he_id1: HalfedgeId, he_id2: HalfedgeId) {
        let he1 = unwrap_or_return!(self.halfedges.get_mut(he_id1), "Halfedge not found");
        he1.twin = Some(he_id2);

        let he2 = unwrap_or_return!(self.halfedges.get_mut(he_id2), "Halfedge not found");
        he2.twin = Some(he_id1);
    }

    /// Changes the outgoing halfedge of a vertex and removes the previous one. Doesn't change anything else.
    pub fn change_outgoing_halfedge(&mut self, vertex_id: VertexId, new_halfedge_id: HalfedgeId) {
        let vertex = unwrap_or_return!(self.vertices.get_mut(vertex_id), "Vertex not found");

        let prev_outgoing_he_id = vertex.outgoing_halfedge;

        vertex.outgoing_halfedge = Some(new_halfedge_id);

        self.outgoing_halfedges
            .entry(vertex_id)
            .unwrap()
            .or_default();

        if let Some(prev_outgoing_he_id) = prev_outgoing_he_id {
            self.outgoing_halfedges[vertex_id].retain(|he_id| *he_id != prev_outgoing_he_id);
        }

        self.outgoing_halfedges[vertex_id].push(new_halfedge_id);
    }
}
