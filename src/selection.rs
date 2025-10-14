use hashbrown::HashSet;
use tracing::{error, instrument};

use super::{FaceId, HalfedgeId, MeshGraph, VertexId};

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Selection {
    pub vertices: HashSet<VertexId>,
    pub halfedges: HashSet<HalfedgeId>,
    pub faces: HashSet<FaceId>,
}

impl Selection {
    pub fn select_all(mesh_graph: &MeshGraph) -> Self {
        Self {
            faces: mesh_graph.faces.keys().collect(),
            ..Default::default()
        }
    }

    pub fn resolve_to_halfedges(&self, mesh_graph: &MeshGraph) -> HashSet<HalfedgeId> {
        let mut halfedges = self.halfedges.clone();

        for face in &self.faces {
            halfedges.extend(mesh_graph.faces[*face].halfedges(mesh_graph));
        }

        for vertex in &self.vertices {
            halfedges.extend(mesh_graph.vertices[*vertex].outgoing_halfedges(mesh_graph));
        }

        halfedges
    }

    pub fn resolve_to_vertices(&self, mesh_graph: &MeshGraph) -> HashSet<VertexId> {
        let mut vertices = self.vertices.clone();

        for halfedge in &self.halfedges {
            let he = mesh_graph.halfedges[*halfedge];
            if let Some(start_vertex) = he.start_vertex(mesh_graph) {
                vertices.insert(start_vertex);
            } else {
                error!("Start vertex not found");
            }
            vertices.insert(he.end_vertex);
        }

        for face in &self.faces {
            vertices.extend(mesh_graph.faces[*face].vertices(mesh_graph));
        }

        vertices
    }

    // TODO : also resolve to faces

    #[instrument(skip(mesh_graph))]
    /// Grows the selection by neighboring vertices. It returns the new vertices.
    pub fn grow(&mut self, mesh_graph: &MeshGraph) -> HashSet<VertexId> {
        let existing_verts = self.resolve_to_vertices(mesh_graph);

        let mut new_verts = HashSet::new();

        for vert_id in &existing_verts {
            if let Some(vert) = mesh_graph.vertices.get(*vert_id) {
                for neighbor in vert.neighbours(mesh_graph) {
                    if !existing_verts.contains(&neighbor) {
                        new_verts.insert(neighbor);
                        self.insert(neighbor);
                    }
                }
            } else {
                error!("Vertex not found");
            }
        }

        new_verts
    }
}

pub trait SelectionOps<T> {
    fn insert(&mut self, item: T);
    fn remove(&mut self, item: T);
}

impl SelectionOps<VertexId> for Selection {
    fn insert(&mut self, item: VertexId) {
        self.vertices.insert(item);
    }

    fn remove(&mut self, item: VertexId) {
        self.vertices.remove(&item);
    }
}

impl SelectionOps<HalfedgeId> for Selection {
    fn insert(&mut self, item: HalfedgeId) {
        self.halfedges.insert(item);
    }

    fn remove(&mut self, item: HalfedgeId) {
        self.halfedges.remove(&item);
    }
}

impl SelectionOps<FaceId> for Selection {
    fn insert(&mut self, item: FaceId) {
        self.faces.insert(item);
    }

    fn remove(&mut self, item: FaceId) {
        self.faces.remove(&item);
    }
}

impl From<VertexId> for Selection {
    fn from(value: VertexId) -> Self {
        Self::from_iter(vec![value])
    }
}

impl From<HalfedgeId> for Selection {
    fn from(value: HalfedgeId) -> Self {
        Self::from_iter(vec![value])
    }
}

impl From<FaceId> for Selection {
    fn from(value: FaceId) -> Self {
        Self::from_iter(vec![value])
    }
}

impl FromIterator<VertexId> for Selection {
    fn from_iter<T: IntoIterator<Item = VertexId>>(iter: T) -> Self {
        Selection {
            vertices: HashSet::from_iter(iter),
            ..Default::default()
        }
    }
}

impl FromIterator<HalfedgeId> for Selection {
    fn from_iter<T: IntoIterator<Item = HalfedgeId>>(iter: T) -> Self {
        Selection {
            halfedges: HashSet::from_iter(iter),
            ..Default::default()
        }
    }
}

impl FromIterator<FaceId> for Selection {
    fn from_iter<T: IntoIterator<Item = FaceId>>(iter: T) -> Self {
        Selection {
            faces: HashSet::from_iter(iter),
            ..Default::default()
        }
    }
}

macro_rules! impl_from_for_selection {
    ($type:ident) => {
        impl From<$type<VertexId>> for Selection {
            fn from(value: $type<VertexId>) -> Self {
                Self::from_iter(value)
            }
        }
        impl From<$type<HalfedgeId>> for Selection {
            fn from(value: $type<HalfedgeId>) -> Self {
                Self::from_iter(value)
            }
        }
        impl From<$type<FaceId>> for Selection {
            fn from(value: $type<FaceId>) -> Self {
                Self::from_iter(value)
            }
        }
    };
}

impl_from_for_selection!(Vec);
impl_from_for_selection!(HashSet);
