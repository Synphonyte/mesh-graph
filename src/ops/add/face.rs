use glam::Vec3;
use tracing::{error, instrument};

use crate::{Face, FaceId, HalfedgeId, MeshGraph, VertexId, error_none};

impl MeshGraph {
    #[instrument(skip(self))]
    pub fn add_face_from_positions(&mut self, a: Vec3, b: Vec3, c: Vec3) -> AddFace {
        let a_id = self.add_vertex(a);
        let b_id = self.add_vertex(b);
        let c_id = self.add_vertex(c);

        // vertices were just inserted, so this can't fail
        let inserted_a = self.add_or_get_edge(a_id, b_id).unwrap();
        let inserted_b = self.add_or_get_edge(b_id, c_id).unwrap();
        let inserted_c = self.add_or_get_edge(c_id, a_id).unwrap();

        let face_id = self.add_face(
            inserted_a.start_to_end_he_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_a.created_he_ids();
        halfedge_ids.extend(inserted_b.created_he_ids());
        halfedge_ids.extend(inserted_c.created_he_ids());

        AddFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![a_id, b_id, c_id],
        }
    }

    /// Returns `None` when an edge already has two faces
    #[instrument(skip(self))]
    pub fn add_face_from_halfedge_and_position(
        &mut self,
        he_id: HalfedgeId,
        opposite_vertex_pos: Vec3,
    ) -> Option<AddFace> {
        let vertex_id = self.add_vertex(opposite_vertex_pos);
        self.add_face_from_halfedge_and_vertex(he_id, vertex_id)
    }

    /// Returns `None` when the edge described by `he_id` already has two faces
    #[instrument(skip(self))]
    pub fn add_face_from_halfedge_and_vertex(
        &mut self,
        he_id: HalfedgeId,
        vertex_id: VertexId,
    ) -> Option<AddFace> {
        let he_a_id = self.boundary_he(he_id)?;

        let he_a = self.halfedges[he_a_id]; // already checked in `boundary_he`
        let start_vertex_id_he_a = he_a.start_vertex(self)?;
        let end_vertex_id_he_a = he_a.end_vertex;

        let inserted_b = self.add_or_get_edge(end_vertex_id_he_a, vertex_id)?;
        let inserted_c = self.add_or_get_edge(vertex_id, start_vertex_id_he_a)?;

        let face_id = self.add_face(
            he_a_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_b.created_he_ids();
        halfedge_ids.extend(inserted_c.created_he_ids());

        Some(AddFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![],
        })
    }

    /// Creates a face from three vertices.
    #[instrument(skip(self))]
    pub fn add_face_from_vertices(
        &mut self,
        v_id1: VertexId,
        v_id2: VertexId,
        v_id3: VertexId,
    ) -> Option<AddFace> {
        let inserted_a = self.add_or_get_edge(v_id1, v_id2)?;
        let inserted_b = self.add_or_get_edge(v_id2, v_id3)?;
        let inserted_c = self.add_or_get_edge(v_id3, v_id1)?;

        let face_id = self.add_face(
            inserted_a.start_to_end_he_id,
            inserted_b.start_to_end_he_id,
            inserted_c.start_to_end_he_id,
        );

        let mut halfedge_ids = inserted_a.created_he_ids();
        halfedge_ids.extend(inserted_b.created_he_ids());
        halfedge_ids.extend(inserted_c.created_he_ids());

        Some(AddFace {
            face_id,
            halfedge_ids,
            vertex_ids: vec![],
        })
    }

    /// Which halfedge of the loop the face that closes it starts at: the one whose start
    /// vertex has the lowest id, or `0` for a loop too short to have a choice.
    ///
    /// A property of the loop, never of the walk that found it or of the order the
    /// candidates were collected in — a mesh that changed with the hasher behind a
    /// `HashSet` would not be reproducible.
    pub(crate) fn fan_corner(&self, loop_hes: &[HalfedgeId]) -> usize {
        if loop_hes.len() <= 3 {
            return 0;
        }

        let mut corner = 0;
        let mut lowest = None;

        for (i, he_id) in loop_hes.iter().enumerate() {
            let Some(start) = self
                .halfedges
                .get(*he_id)
                .and_then(|he| he.start_vertex(self))
            else {
                continue;
            };

            if lowest.is_none_or(|lowest| start < lowest) {
                lowest = Some(start);
                corner = i;
            }
        }

        corner
    }

    /// Newell normal of the loop's polygon (area-weighted, so unaffected by concavity).
    fn loop_normal(&self, loop_hes: &[HalfedgeId]) -> Vec3 {
        let mut normal = Vec3::ZERO;

        for he_id in loop_hes {
            let Some(he) = self.halfedges.get(*he_id) else {
                continue;
            };
            let (Some(a), Some(b)) = (
                he.start_vertex(self).and_then(|v| self.positions.get(v)),
                self.positions.get(he.end_vertex),
            ) else {
                continue;
            };

            normal += a.cross(*b);
        }

        normal
    }

    /// Whether the triangle over `he_a`/`he_b` is an acceptable cut of the loop: it faces the
    /// same way as the loop and is not a sliver, and - with `check_containment`, which makes
    /// it an ear - contains none of the loop's other vertices, so the rest stays a simple
    /// polygon. A loop with no usable normal has no geometry to judge, so nothing is vetoed.
    fn cut_is_an_ear(
        &self,
        loop_hes: &[HalfedgeId],
        he_a: HalfedgeId,
        he_b: HalfedgeId,
        loop_normal: Vec3,
        check_containment: bool,
    ) -> bool {
        let (Some(a), Some(b)) = (self.halfedges.get(he_a), self.halfedges.get(he_b)) else {
            return true;
        };
        let (Some(p0), Some(p1), Some(p2)) = (
            a.start_vertex(self).and_then(|v| self.positions.get(v)),
            self.positions.get(a.end_vertex),
            self.positions.get(b.end_vertex),
        ) else {
            return true;
        };
        let (p0, p1, p2) = (*p0, *p1, *p2);

        let tri_normal = (p1 - p0).cross(p2 - p1);

        if loop_normal.length_squared() < f32::EPSILON {
            return true;
        }
        if tri_normal.length_squared() < 1e-12 || tri_normal.dot(loop_normal) <= 0.0 {
            return false;
        }

        if !check_containment {
            return true;
        }

        // Inside or on the boundary: a vertex lying on the diagonal would leave a remainder
        // that touches itself. A vertex *at* a corner's position is the corner, not an
        // intruder - coincident vertices are what the weld's contact holes are made of.
        let inside = |p: Vec3| {
            [(p0, p1), (p1, p2), (p2, p0)]
                .iter()
                .all(|(from, to)| (*to - *from).cross(p - *from).dot(loop_normal) >= 0.0)
        };
        let ends = [a.start_vertex(self), Some(a.end_vertex), Some(b.end_vertex)];

        !loop_hes.iter().any(|he_id| {
            self.halfedges.get(*he_id).is_some_and(|he| {
                !ends.contains(&Some(he.end_vertex))
                    && self
                        .positions
                        .get(he.end_vertex)
                        .is_some_and(|p| ![p0, p1, p2].contains(p) && inside(*p))
            })
        })
    }

    /// Whether the face over `he_a`/`he_b` can be added without inventing a second edge
    /// between the same two vertices.
    ///
    /// `add_face_from_halfedges` closes the loop with the edge `he_b.end -> he_a.start` and
    /// reuses an existing edge only when that halfedge is a boundary one; a *faced* halfedge
    /// running that way makes the helper add a parallel edge instead, which is a
    /// non-manifold duplicate. The corner is skipped rather than the mesh corrupted.
    pub(crate) fn can_close_loop_with(&self, he_a: HalfedgeId, he_b: HalfedgeId) -> bool {
        let (Some(start), Some(end)) = (
            self.halfedges
                .get(he_a)
                .and_then(|he| he.start_vertex(self)),
            self.halfedges.get(he_b).map(|he| he.end_vertex),
        ) else {
            return false;
        };

        let existing = self.halfedges_from_to(end, start);

        existing.is_empty()
            || existing.iter().any(|he_id| {
                self.halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.is_boundary() && he.twin.is_some())
            })
    }

    /// The face on the far side of a three-halfedge boundary loop, when all three twins belong
    /// to it: a lone triangle, attached to nothing but the hole around it.
    ///
    /// Closing such a loop would add the reverse face and leave a closed two-face pillow of
    /// zero volume - watertight, but degenerate, and exactly the coincident pair a later flap
    /// cleanup deletes again. The hole closers remove the lone face instead, which leaves no
    /// boundary either.
    pub(crate) fn lone_face_behind(&self, loop_hes: &[HalfedgeId]) -> Option<FaceId> {
        let [he_a, he_b, he_c] = loop_hes else {
            return None;
        };

        let face_behind = |he_id: HalfedgeId| -> Option<FaceId> {
            let he = self.halfedges.get(he_id)?;
            if he.face.is_some() {
                return None;
            }
            self.halfedges.get(he.twin?)?.face
        };

        let face_id = face_behind(*he_a)?;
        (face_behind(*he_b) == Some(face_id) && face_behind(*he_c) == Some(face_id))
            .then_some(face_id)
    }

    /// Closes a loop of boundary halfedges with one face, rotating the fan's corner from
    /// [`Self::fan_corner`] until a cut is possible.
    ///
    /// The first corner is a property of the loop; the rotation only moves on when that
    /// corner cannot be cut — its diagonal already exists as an interior edge, so
    /// `add_face_from_halfedges` adds nothing or would pair two vertices that are already
    /// joined. One face at a time: a loop of `n` halfedges is closed by `n - 2` calls, each
    /// leaving the triangle beyond the cut for the next one.
    pub(crate) fn close_loop_with_fan(&mut self, loop_hes: &[HalfedgeId]) -> Option<AddFace> {
        debug_assert!(
            self.lone_face_behind(loop_hes).is_none(),
            "closing {loop_hes:?} would build a pillow over a lone triangle"
        );

        let len = loop_hes.len();
        if len < 3 {
            return None;
        }

        // Three *boundary* halfedges are a face already: all three of the loop's edges exist, so
        // the face goes in over them. Asking `add_face_from_halfedges` to do it instead makes it
        // acquire the third edge, which fails whenever that edge already exists in the other
        // direction - the weld's small-gap triangles were left open one face short by exactly
        // that. Guarded on all three being boundary: a halfedge the walk reached through the
        // opened-set alone may already carry a face, and adding over it would not shrink the
        // boundary, i.e. would not make progress.
        if len == 3
            && loop_hes.iter().all(|he_id| {
                self.halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.face.is_none())
            })
        {
            // When the three twins are all one face, this would close a zero-volume two-face
            // pillow; the closers check [`Self::lone_face_behind`] first and drop that face
            // instead.
            let face_id = self.add_face(loop_hes[0], loop_hes[1], loop_hes[2]);

            return Some(AddFace {
                face_id,
                halfedge_ids: Vec::new(),
                vertex_ids: Vec::new(),
            });
        }

        let first = self.fan_corner(loop_hes);
        let loop_normal = self.loop_normal(loop_hes);

        // Three tiers, each tried only when the one before finds no corner:
        //  0. an ear: the triangle faces the way the loop does, is not a sliver and holds no
        //     other vertex of the loop, so a cut across a non-convex loop cannot fold over the
        //     surface and the rest stays a simple polygon;
        //  1. a correctly facing, non-sliver triangle - what is left when the loop has vertices
        //     in odd places (the weld's contact holes have several at one position);
        //  2. any cut, because a hole left open is worse than a poor triangle.
        for tier in 0..3 {
            for step in 0..len {
                let corner = (first + step) % len;
                let (he_a, he_b) = (loop_hes[corner], loop_hes[(corner + 1) % len]);

                if !self.can_close_loop_with(he_a, he_b) {
                    continue;
                }
                if tier < 2 && !self.cut_is_an_ear(loop_hes, he_a, he_b, loop_normal, tier == 0) {
                    continue;
                }

                if let Some(added) = self.add_face_from_halfedges(he_a, he_b) {
                    return Some(added);
                }
            }
        }

        None
    }

    /// Creates a face from two halfedges.
    #[instrument(skip(self))]
    pub fn add_face_from_halfedges(
        &mut self,
        he_id1: HalfedgeId,
        he_id2: HalfedgeId,
    ) -> Option<AddFace> {
        let he_id1 = self
            .boundary_he(he_id1)
            .or_else(error_none!("Boundary Halfedge 1 {he_id1:?} not found"))?;
        let he_id2 = self
            .boundary_he(he_id2)
            .or_else(error_none!("Boundary Halfedge 2 {he_id2:?} not found"))?;

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
            let inserted = self.add_or_get_boundary_edge(he1.end_vertex, he2_start_vertex)?;

            let face_id = self.add_face(inserted.start_to_end_he_id, he_id2, he_id1);

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else if he1_start_vertex == he2_start_vertex {
            let inserted = self.add_or_get_boundary_edge(he1.end_vertex, he2.end_vertex)?;

            let face_id = self.add_face(
                inserted.start_to_end_he_id,
                he2.twin.or_else(error_none!("Twin not set"))?,
                he_id1,
            );

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else if he1.end_vertex == he2.end_vertex {
            let inserted = self.add_or_get_boundary_edge(he2_start_vertex, he1_start_vertex)?;

            let face_id = self.add_face(
                inserted.start_to_end_he_id,
                he_id1,
                he2.twin.or_else(error_none!("Twin not set"))?,
            );

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        } else {
            let inserted = self.add_or_get_boundary_edge(he2.end_vertex, he1_start_vertex)?;

            let face_id = self.add_face(inserted.start_to_end_he_id, he_id1, he_id2);

            Some(AddFace {
                face_id,
                halfedge_ids: inserted.created_he_ids(),
                vertex_ids: vec![],
            })
        }
    }

    /// Inserts a face into the mesh graph. It connects the halfedges to the face and the face to the first halfedge.
    /// Additionally it connects the halfedges' `next` loop around the face.
    #[instrument(skip(self))]
    pub fn add_face(
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

        let face = self.faces[face_id]; // just inserted above
        self.bvh
            .insert_or_update_partially(face.aabb(self), face.index, 0.0);

        face_id
    }
}

/// Return value of several `add_face...` methods
pub struct AddFace {
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

    fn add_face(mesh_graph: &mut MeshGraph) -> FaceId {
        mesh_graph
            .add_face_from_positions(
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
            .add_face_from_halfedge_and_position(associated_he_id, pos)
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
            .add_face_from_halfedges(he_id_1, he_id_2)
            .map(|f| f.face_id)
    }

    macro_rules! init_fill_face {
        ($f1:ident, $f2:ident, $f3:ident, $mg:ident) => {
            let $f1 = add_face(&mut $mg);

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
        let face1 = add_face(&mut mesh_graph);

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
        let face1 = add_face(&mut mesh_graph);

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
        let face1 = add_face(&mut mesh_graph);

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

#[cfg(test)]
mod tests {
    use glam::{Vec3, vec3};

    use crate::*;

    /// An annulus around an L-shaped (non-convex) hole. `rotation` shifts which inner vertex
    /// gets the lowest id, which decides where the fan starts cutting.
    fn l_hole_annulus(rotation: usize) -> (MeshGraph, Vec3, Vec<VertexId>) {
        annulus_around(
            &[
                vec3(0.0, 0.0, 0.0),
                vec3(2.0, 0.0, 0.0),
                vec3(2.0, 1.0, 0.0),
                vec3(1.0, 1.0, 0.0),
                vec3(1.0, 2.0, 0.0),
                vec3(0.0, 2.0, 0.0),
            ],
            rotation,
        )
    }

    /// An annulus around a hole with the given counter-clockwise outline, star-shaped about
    /// (0.5, 0.5).
    fn annulus_around(inner: &[Vec3], rotation: usize) -> (MeshGraph, Vec3, Vec<VertexId>) {
        // The L is star-shaped about the centre of its lower-left unit square.
        let centre = vec3(0.5, 0.5, 0.0);
        let n = inner.len();
        let rotation = rotation % n;

        let mut mesh = MeshGraph::new();
        let mut inner_ids = vec![None; n];
        let mut outer_ids = vec![None; n];
        for step in 0..n {
            let i = (step + rotation) % n;
            inner_ids[i] = Some(mesh.add_vertex(inner[i]));
        }
        for step in 0..n {
            let i = (step + rotation) % n;
            outer_ids[i] = Some(mesh.add_vertex(centre + (inner[i] - centre) * 2.0));
        }

        for i in 0..n {
            let j = (i + 1) % n;
            let (ii, ij) = (inner_ids[i].unwrap(), inner_ids[j].unwrap());
            let (oi, oj) = (outer_ids[i].unwrap(), outer_ids[j].unwrap());
            mesh.add_face_from_vertices(ij, ii, oj).unwrap();
            mesh.add_face_from_vertices(ii, oi, oj).unwrap();
        }

        // Every annulus face has this normal; the fill of a flat sheet must share it.
        let first = mesh.faces.values().next().unwrap();
        let normal = face_normal(&mesh, first.halfedge);

        (mesh, normal, inner_ids.into_iter().flatten().collect())
    }

    fn face_normal(mesh: &MeshGraph, start: HalfedgeId) -> Vec3 {
        let h0 = mesh.halfedges[start];
        let h1 = mesh.halfedges[h0.next.unwrap()];
        let h2 = mesh.halfedges[h1.next.unwrap()];
        let p0 = mesh.positions[h0.start_vertex(mesh).unwrap()];
        let p1 = mesh.positions[h1.start_vertex(mesh).unwrap()];
        let p2 = mesh.positions[h2.start_vertex(mesh).unwrap()];
        (p1 - p0).cross(p2 - p1)
    }

    /// The boundary loop of the hole: the one that runs through the `inner` vertices.
    fn boundary_loop(mesh: &MeshGraph, inner: &[VertexId]) -> Vec<HalfedgeId> {
        let boundary: Vec<HalfedgeId> = mesh
            .halfedges
            .iter()
            .filter(|(_, he)| he.face.is_none())
            .map(|(id, _)| id)
            .collect();
        let Some(&first) = boundary
            .iter()
            .filter(|id| {
                mesh.halfedges[**id]
                    .start_vertex(mesh)
                    .is_some_and(|v| inner.contains(&v))
            })
            .min()
        else {
            return Vec::new();
        };

        let mut loop_hes = vec![first];
        loop {
            let end = mesh.halfedges[*loop_hes.last().unwrap()].end_vertex;
            let next = *boundary
                .iter()
                .find(|id| mesh.halfedges[**id].start_vertex(mesh) == Some(end))
                .expect("a simple hole has a continuation");
            if next == first {
                return loop_hes;
            }
            loop_hes.push(next);
        }
    }

    /// Fanning a six-sided L-shaped hole must not fold a triangle over the surface, whichever
    /// vertex has the lowest id (that is where the fan starts cutting).
    #[test]
    fn test_close_loop_with_fan_does_not_fold_a_non_convex_hole() {
        for rotation in 0..6 {
            let (mut mesh, normal, inner) = l_hole_annulus(rotation);
            let faces_before = mesh.faces.len();

            for _ in 0..8 {
                let loop_hes = boundary_loop(&mesh, &inner);
                if loop_hes.is_empty() {
                    break;
                }
                assert!(
                    mesh.close_loop_with_fan(&loop_hes).is_some(),
                    "rotation {rotation}: no cut possible for {} halfedges",
                    loop_hes.len()
                );
            }

            assert!(
                boundary_loop(&mesh, &inner).is_empty(),
                "rotation {rotation}: hole not closed"
            );
            assert_eq!(
                mesh.faces.len(),
                faces_before + 4,
                "L hexagon = 4 triangles"
            );

            for face in mesh.faces.values() {
                assert!(
                    face_normal(&mesh, face.halfedge).dot(normal) > 0.0,
                    "rotation {rotation}: face {:?} is flipped",
                    face.id
                );
            }
        }
    }

    /// Vertices at one position - what the weld's contact holes contain - must not make every
    /// ear look occupied: the hole still closes, and no triangle faces the wrong way.
    #[test]
    fn test_close_loop_with_fan_copes_with_coincident_loop_vertices() {
        let outline = [
            vec3(0.0, 0.0, 0.0),
            vec3(0.0, 0.0, 0.0),
            vec3(2.0, 0.0, 0.0),
            vec3(2.0, 1.0, 0.0),
            vec3(1.0, 1.0, 0.0),
            vec3(1.0, 2.0, 0.0),
            vec3(0.0, 2.0, 0.0),
        ];

        for rotation in 0..outline.len() {
            let (mut mesh, normal, inner) = annulus_around(&outline, rotation);

            for _ in 0..10 {
                let loop_hes = boundary_loop(&mesh, &inner);
                if loop_hes.is_empty() {
                    break;
                }
                assert!(mesh.close_loop_with_fan(&loop_hes).is_some());
            }

            assert!(
                boundary_loop(&mesh, &inner).is_empty(),
                "rotation {rotation}: open"
            );
            for face in mesh.faces.values() {
                assert!(
                    face_normal(&mesh, face.halfedge).dot(normal) >= 0.0,
                    "rotation {rotation}: face {:?} is flipped",
                    face.id
                );
            }
        }
    }
}
