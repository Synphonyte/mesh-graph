use tracing::error;

use crate::{HalfedgeId, MeshGraph};

/// Iterator over some halfedges
pub struct CircularHalfedgesIterator<'a> {
    start_halfedge: Option<HalfedgeId>,
    mesh_graph: &'a MeshGraph,
    current_halfedge: Option<HalfedgeId>,
    get_next_halfedge: fn(HalfedgeId, &'a MeshGraph) -> Option<HalfedgeId>,
    max_count: usize,
    next_idx: usize,
}

impl<'a> CircularHalfedgesIterator<'a> {
    pub fn new(
        start_halfedge: Option<HalfedgeId>,
        mesh_graph: &'a MeshGraph,
        get_next_halfedge: fn(HalfedgeId, &'a MeshGraph) -> Option<HalfedgeId>,
        max_count: usize,
    ) -> Self {
        Self {
            start_halfedge,
            mesh_graph,
            current_halfedge: None,
            get_next_halfedge,
            max_count,
            next_idx: 0,
        }
    }

    pub fn empty(mesh_graph: &'a MeshGraph) -> Self {
        Self {
            start_halfedge: None,
            mesh_graph,
            current_halfedge: None,
            get_next_halfedge: |_, _| None,
            max_count: 10,
            next_idx: 0,
        }
    }
}

impl<'a> Iterator for CircularHalfedgesIterator<'a> {
    type Item = HalfedgeId;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(current_halfedge) = self.current_halfedge {
            self.current_halfedge = (self.get_next_halfedge)(current_halfedge, self.mesh_graph);
            self.next_idx += 1;

            if self.current_halfedge == self.start_halfedge {
                return None;
            }
        } else {
            self.current_halfedge = self.start_halfedge;
            self.next_idx += 1;
        }

        if self.next_idx > self.max_count {
            error!(
                "Maximum count reached. In a valid mesh graph this should not happen. Start halfedge: {:?}",
                self.start_halfedge
            );
            return None;
        }

        self.current_halfedge
    }
}
