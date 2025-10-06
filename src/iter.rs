use crate::{HalfedgeId, MeshGraph};

/// Iterator over some halfedges
pub struct CircularHalfedgesIterator<'a> {
    start_halfedge: Option<HalfedgeId>,
    mesh_graph: &'a MeshGraph,
    current_halfedge: Option<HalfedgeId>,
    get_next_halfedge: fn(HalfedgeId, &'a MeshGraph) -> Option<HalfedgeId>,
}

impl<'a> CircularHalfedgesIterator<'a> {
    pub fn new(
        start_halfedge: Option<HalfedgeId>,
        mesh_graph: &'a MeshGraph,
        get_next_halfedge: fn(HalfedgeId, &'a MeshGraph) -> Option<HalfedgeId>,
    ) -> Self {
        Self {
            start_halfedge,
            mesh_graph,
            current_halfedge: None,
            get_next_halfedge,
        }
    }
}

impl<'a> Iterator for CircularHalfedgesIterator<'a> {
    type Item = HalfedgeId;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(current_halfedge) = self.current_halfedge {
            self.current_halfedge = (self.get_next_halfedge)(current_halfedge, self.mesh_graph);

            if self.current_halfedge == self.start_halfedge {
                return None;
            }
        } else {
            self.current_halfedge = self.start_halfedge;
        }

        self.current_halfedge
    }
}
