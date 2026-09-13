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

    /// Resolves the selection to the halfedges it covers: the explicitly selected
    /// ones, every halfedge of a selected face, and every outgoing halfedge of a
    /// selected vertex.
    ///
    /// Ids that are no longer live are skipped rather than panicking — a `Selection`
    /// holds bare keys and any topology change can invalidate them. Use
    /// [`Self::retain_live`] to drop them.
    pub fn resolve_to_halfedges(&self, mesh_graph: &MeshGraph) -> HashSet<HalfedgeId> {
        let mut halfedges = self.halfedges.clone();

        for face in &self.faces {
            let Some(face) = mesh_graph.faces.get(*face) else {
                error!("Face not found");
                continue;
            };
            halfedges.extend(face.halfedges(mesh_graph));
        }

        for vertex in &self.vertices {
            let Some(vertex) = mesh_graph.vertices.get(*vertex) else {
                error!("Vertex not found");
                continue;
            };
            halfedges.extend(vertex.outgoing_halfedges(mesh_graph));
        }

        halfedges
    }

    pub fn resolve_to_vertices(&self, mesh_graph: &MeshGraph) -> HashSet<VertexId> {
        let mut vertices = self.vertices.clone();

        for halfedge in &self.halfedges {
            let Some(he) = mesh_graph.halfedges.get(*halfedge) else {
                error!("Halfedge not found");
                continue;
            };
            if let Some(start_vertex) = he.start_vertex(mesh_graph) {
                vertices.insert(start_vertex);
            } else {
                error!("Start vertex not found");
            }
            vertices.insert(he.end_vertex);
        }

        for face in &self.faces {
            let Some(face) = mesh_graph.faces.get(*face) else {
                error!("Face not found");
                continue;
            };
            vertices.extend(face.vertices(mesh_graph));
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

    /// Drops every id that is no longer live in `mesh_graph`, returning exactly which
    /// ones were dropped.
    ///
    /// This is how a [`Selection`] survives a topology change. Liveness is read from
    /// the mesh itself rather than from a record of what each operation removed, so it
    /// cannot miss a removal site — see [`SelectionEdit`].
    pub fn retain_live(&mut self, mesh_graph: &MeshGraph) -> SelectionEdit {
        let mut removed = SelectionEdit::default();

        self.vertices.retain(|v_id| {
            let live = mesh_graph.vertices.contains_key(*v_id);
            if !live {
                removed.vertices.push(*v_id);
            }
            live
        });
        self.halfedges.retain(|he_id| {
            let live = mesh_graph.halfedges.contains_key(*he_id);
            if !live {
                removed.halfedges.push(*he_id);
            }
            live
        });
        self.faces.retain(|f_id| {
            let live = mesh_graph.faces.contains_key(*f_id);
            if !live {
                removed.faces.push(*f_id);
            }
            live
        });

        removed
    }

    /// Adds every element of `edit` to the matching set.
    pub fn extend_with_edit(&mut self, edit: &SelectionEdit) {
        self.vertices.extend(edit.vertices.iter().copied());
        self.halfedges.extend(edit.halfedges.iter().copied());
        self.faces.extend(edit.faces.iter().copied());
    }

    /// Removes every element of `edit` from the matching set.
    pub fn remove_edit(&mut self, edit: &SelectionEdit) {
        for v_id in &edit.vertices {
            self.vertices.remove(v_id);
        }
        for he_id in &edit.halfedges {
            self.halfedges.remove(he_id);
        }
        for f_id in &edit.faces {
            self.faces.remove(f_id);
        }
    }
}

/// A set of mesh elements a scoped operation added to, or removed from, a
/// [`Selection`].
///
/// Applying a scoped operation's `removed` and then its `added` to the selection it
/// started from reproduces the selection it ended with, so a caller mirroring the
/// selection elsewhere can follow along without re-scanning the mesh.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SelectionEdit {
    pub vertices: Vec<VertexId>,
    pub halfedges: Vec<HalfedgeId>,
    pub faces: Vec<FaceId>,
}

impl SelectionEdit {
    /// True when nothing was added or removed.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty() && self.halfedges.is_empty() && self.faces.is_empty()
    }

    /// Total number of elements across all three kinds.
    pub fn len(&self) -> usize {
        self.vertices.len() + self.halfedges.len() + self.faces.len()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::build_grid;

    /// `select_all` seeds *faces only*, so every id `resolve_to_halfedges` and
    /// `resolve_to_vertices` hand back here is derived (a selected face's own
    /// halfedges/vertices), never one of the selection's unfiltered explicit ids.
    /// That is what makes the strict "every resolved id is live" assertion
    /// legitimate for this fixture. Panics on unfixed code, at the first stale face.
    #[test]
    fn test_resolve_tolerates_dead_faces_in_the_selection() {
        let mut mg = build_grid(4);
        let selection = Selection::select_all(&mg);

        mg.collapse_until_edges_above_min_length(1.5, &mut HashSet::new());

        assert!(
            selection.faces.iter().any(|f_id| !mg.faces.contains_key(*f_id)),
            "fixture failed to invalidate anything - test is vacuous"
        );

        for he_id in selection.resolve_to_halfedges(&mg) {
            assert!(
                mg.halfedges.contains_key(he_id),
                "resolved halfedge {he_id:?} is dead"
            );
        }
        for v_id in selection.resolve_to_vertices(&mg) {
            assert!(
                mg.vertices.contains_key(v_id),
                "resolved vertex {v_id:?} is dead"
            );
        }
    }

    /// A selection naming every vertex, halfedge and face reaches the stale-halfedge
    /// and stale-vertex lookup sites the faces-only fixture above cannot. Both
    /// resolves must complete without panicking, and every id that was still live in
    /// the selection must still appear in the corresponding result - the unfiltered
    /// seed (`self.vertices.clone()` / `self.halfedges.clone()`) passes live explicit
    /// ids straight through by design, and this pins that behaviour rather than
    /// asserting every resolved id is live (which is false here on purpose).
    #[test]
    fn test_resolve_keeps_live_ids_when_the_selection_is_stale() {
        let mut mg = build_grid(4);
        let selection = Selection {
            vertices: mg.vertices.keys().collect(),
            halfedges: mg.halfedges.keys().collect(),
            faces: mg.faces.keys().collect(),
        };

        mg.collapse_until_edges_above_min_length(1.5, &mut HashSet::new());

        let live_vertices_before: HashSet<VertexId> = selection
            .vertices
            .iter()
            .copied()
            .filter(|v_id| mg.vertices.contains_key(*v_id))
            .collect();
        let live_halfedges_before: HashSet<HalfedgeId> = selection
            .halfedges
            .iter()
            .copied()
            .filter(|he_id| mg.halfedges.contains_key(*he_id))
            .collect();

        assert!(
            live_vertices_before.len() < selection.vertices.len(),
            "fixture failed to invalidate anything - test is vacuous"
        );

        let resolved_halfedges = selection.resolve_to_halfedges(&mg);
        let resolved_vertices = selection.resolve_to_vertices(&mg);

        for v_id in &live_vertices_before {
            assert!(
                resolved_vertices.contains(v_id),
                "live vertex {v_id:?} missing from resolve_to_vertices"
            );
        }
        for he_id in &live_halfedges_before {
            assert!(
                resolved_halfedges.contains(he_id),
                "live halfedge {he_id:?} missing from resolve_to_halfedges"
            );
        }
    }
}
