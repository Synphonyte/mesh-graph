use tracing::{error, instrument};

use crate::{Halfedge, HalfedgeId, MeshGraph, VertexId, error_none};

impl MeshGraph {
    /// Inserts a pair of halfedges and connects them to the given vertices (from and to the halfedge) and each other as twins.
    /// If the edge already exists or partially exists,
    /// it returns the existing edge while creating any missing halfedges.
    ///
    /// See also [`add_edge`].
    #[instrument(skip(self))]
    pub fn add_or_get_edge(
        &mut self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Option<AddOrGetEdge> {
        let ret = match self.add_or_get_edge_inner(start_vertex_id, end_vertex_id) {
            Some(ret) => ret,
            None => {
                let start_to_end_he_id = self.add_halfedge(start_vertex_id, end_vertex_id)?;
                let twin_he_id = self.add_halfedge(end_vertex_id, start_vertex_id)?;

                AddOrGetEdge {
                    start_to_end_he_id,
                    twin_he_id,
                    new_start_to_end: true,
                    new_twin: true,
                }
            }
        };

        // insert_or_get_edge_inner ensures that both halfedges exist.
        self.halfedges[ret.start_to_end_he_id].twin = Some(ret.twin_he_id);
        self.halfedges[ret.twin_he_id].twin = Some(ret.start_to_end_he_id);

        let start_v = self
            .vertices
            .get_mut(start_vertex_id)
            .or_else(error_none!("Vertex not found"))?;
        start_v.outgoing_halfedge = Some(ret.start_to_end_he_id);

        let end_v = self
            .vertices
            .get_mut(end_vertex_id)
            .or_else(error_none!("Vertex not found"))?;
        end_v.outgoing_halfedge = Some(ret.twin_he_id);

        Some(ret)
    }

    /// Adds or gets a boundary edge between two vertices.
    /// If the edge already exists and the halfedge (start_vertex_id, end_vertex_id) is a boundary halfedge, it is returned;
    /// otherwise, a new halfedge pair is created.
    ///
    /// Note that this can lead to duplicate halfedges if the edge already exists but is not a boundary edge.
    pub fn add_or_get_boundary_edge(
        &mut self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Option<AddOrGetEdge> {
        let existing_he_ids = self.halfedges_from_to(start_vertex_id, end_vertex_id);
        if let Some(he_id) = existing_he_ids.into_iter().find(|he_id| {
            // already checked in `halfedges_from_to()`
            let he = &self.halfedges[*he_id];
            he.is_boundary() && he.twin.is_some()
        }) {
            // already checked in `halfedges_from_to()`
            let he = self.halfedges[he_id];

            return Some(AddOrGetEdge {
                start_to_end_he_id: he_id,
                twin_he_id: he.twin.unwrap(), // checked above
                new_start_to_end: false,
                new_twin: false,
            });
        }

        let AddEdge {
            start_to_end_he_id,
            twin_he_id,
        } = self.add_edge(start_vertex_id, end_vertex_id)?;

        Some(AddOrGetEdge {
            start_to_end_he_id,
            twin_he_id,
            new_start_to_end: true,
            new_twin: true,
        })
    }

    fn add_or_get_edge_inner(
        &mut self,
        start_vertex_id: VertexId,
        end_vertex_id: VertexId,
    ) -> Option<AddOrGetEdge> {
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
                he2_id = Some(self.add_halfedge(end_vertex_id, start_vertex_id)?);
                new2 = true;
            }
            (None, Some(_h2_id)) => {
                he1_id = Some(self.add_halfedge(start_vertex_id, end_vertex_id)?);
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

        Some(AddOrGetEdge {
            start_to_end_he_id,
            twin_he_id,
            new_start_to_end: new1,
            new_twin: new2,
        })
    }

    /// Inserts a pair of halfedges and connects them to the given vertices (from and to the halfedge) and each other as twins.
    /// This creates two halfedges wether the vertices already have an edge between them or not.
    ///
    /// If you don't want this, consider using [`add_or_get_edge`] instead.
    pub fn add_edge(&mut self, start_vertex: VertexId, end_vertex: VertexId) -> Option<AddEdge> {
        let start_to_end_he_id = self.add_halfedge(start_vertex, end_vertex)?;
        let twin_he_id = self.add_halfedge(end_vertex, start_vertex)?;

        // Just inserted above
        self.halfedges[start_to_end_he_id].twin = Some(twin_he_id);
        self.halfedges[twin_he_id].twin = Some(start_to_end_he_id);

        // Vertex existence already checked in `add_halfedge`
        self.vertices[start_vertex].outgoing_halfedge = Some(start_to_end_he_id);
        self.vertices[end_vertex].outgoing_halfedge = Some(twin_he_id);

        Some(AddEdge {
            start_to_end_he_id,
            twin_he_id,
        })
    }

    /// Inserts a halfedge into the mesh graph. It only connects the halfedge to the given end vertex but not the reverse.
    /// It also doesn't do any other connections.
    ///
    /// It does insert into `self.outgoing_halfedges`.
    ///
    /// Use [`insert_or_get_edge`] instead of this when you can to lower the chance of creating an invalid graph.
    #[instrument(skip(self))]
    pub fn add_halfedge(
        &mut self,
        start_vertex: VertexId,
        end_vertex: VertexId,
    ) -> Option<HalfedgeId> {
        let halfedge = Halfedge {
            end_vertex,
            next: None,
            twin: None,
            face: None,
        };
        let he_id = self.halfedges.insert(halfedge);

        self.outgoing_halfedges
            .entry(start_vertex)
            .or_else(error_none!("Start vertex not found {start_vertex:?}"))?
            .or_default()
            .push(he_id);

        Some(he_id)
    }
}

/// Return value of `add_or_get_edge`
pub struct AddOrGetEdge {
    /// Id of the halfedge from start vertex to end vertex
    pub start_to_end_he_id: HalfedgeId,
    /// Id of the halfedge from end vertex to start vertex
    pub twin_he_id: HalfedgeId,
    /// Whether the halfedge from start vertex to end vertex was newly created
    pub new_start_to_end: bool,
    /// Whether the halfedge from end vertex to start vertex was newly created
    pub new_twin: bool,
}

impl AddOrGetEdge {
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

/// Return value of `add_edge`
pub struct AddEdge {
    /// Id of the halfedge from start vertex to end vertex
    pub start_to_end_he_id: HalfedgeId,
    /// Id of the halfedge from end vertex to start vertex
    pub twin_he_id: HalfedgeId,
}
