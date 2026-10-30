use itertools::Itertools;
use tracing::{error, instrument};

use crate::{FaceId, HalfedgeId, MeshGraph, VertexId};

/// Append `id` unless it is already in `list`. Used where a small collection has
/// to stay free of duplicates *and* keep a deterministic order (a `HashSet`
/// would give the first but not the second).
pub(crate) fn push_unique<T: PartialEq>(list: &mut Vec<T>, id: T) {
    if !list.contains(&id) {
        list.push(id);
    }
}

impl MeshGraph {
    /// Deletes a face from the mesh graph.
    ///
    /// It also deletes the vertices and halfedges that are no
    /// longer connected to any other faces.
    ///
    /// Returns the ids of the removed vertices and halfedges.
    #[instrument(skip(self))]
    pub fn remove_face(&mut self, face_id: FaceId) -> (Vec<VertexId>, Vec<HalfedgeId>) {
        if !self.faces.contains_key(face_id) {
            return (vec![], vec![]);
        }

        let halfedges = self
            .face_members(face_id)
            .into_iter()
            .map(|he_id| (he_id, self.halfedges[he_id]))
            .collect_vec();

        #[cfg(feature = "instrumentation")]
        crate::record_op_trace!(
            "remove_face({face_id:?}) members={:?}",
            halfedges.iter().map(|(id, _)| id).collect_vec()
        );

        // Ordered, not a `HashSet`: this list decides the order in which the
        // halfedges are freed from the slotmap (and therefore which ids the
        // next allocations reuse), which vertices are visited below, and what
        // the caller gets back. Draining a set here would make all of that
        // depend on the set's iteration order rather than on the mesh. The
        // members come back in key order, so keeping that order makes them a
        // pure function of the mesh.
        let mut removed_halfedges: Vec<HalfedgeId> = Vec::with_capacity(4);
        for (he_id, he) in halfedges {
            // A twinless member (left over by a re-pair sever or a dangling-twin clear)
            // must not abort the face removal: remove the member itself. Same for a
            // member whose twin reference is dead.
            let Some(twin_id) = he.twin else {
                push_unique(&mut removed_halfedges, he_id);
                continue;
            };
            let Some(twin) = self.halfedges.get(twin_id) else {
                push_unique(&mut removed_halfedges, he_id);
                continue;
            };
            if twin.is_boundary() {
                push_unique(&mut removed_halfedges, he_id);
                push_unique(&mut removed_halfedges, twin_id);
            } else if twin.twin != Some(he_id) {
                push_unique(&mut removed_halfedges, he_id);
            } else {
                // already checked above
                self.halfedges[he_id].face = None;
                self.halfedges[he_id].next = None;
            }
        }

        #[cfg(feature = "rerun")]
        self.log_hes_rerun("deleted_by_face_deletion", &removed_halfedges);

        let mut removed_vertices = Vec::with_capacity(3);
        // Ordered for the same reason as `removed_halfedges`: the visit order
        // below decides which vertices are freed first and hence the returned
        // `removed_vertices` order.
        let mut touched_start_vertices: Vec<VertexId> = Vec::with_capacity(3);

        // Update connections from vertices to deleted halfedges
        for &he_id in &removed_halfedges {
            // The start of a removed member is derived from its twin; for a member
            // whose twin is missing/dead/twinless it cannot be resolved. Skipping the
            // cleanup leaves a stale entry in `outgoing_halfedges` that the periodic
            // `contains_key` retain passes purge; aborting the removal instead would
            // leave the face half-detached (some members already nulled) and break
            // its chain, which is the layer-4 corruption.
            let Some(start_v_id) = self.halfedges[he_id].start_vertex(self) else {
                continue;
            };
            if !touched_start_vertices.contains(&start_v_id) {
                touched_start_vertices.push(start_v_id);
            }
            if let Some(out_he_ids) = self.outgoing_halfedges.get_mut(start_v_id) {
                out_he_ids.retain(|&id| id != he_id);
            } else {
                error!("No outgoing halfedges found for vertex {start_v_id:?}");
            }
        }

        for start_v_id in touched_start_vertices.iter().copied() {
            // A vertex survives if it still has a live outgoing halfedge; in that
            // case refresh its seed pointer. Otherwise it is now isolated.
            let surviving_out_he = self
                .outgoing_halfedges
                .get(start_v_id)
                .and_then(|out_he_ids| out_he_ids.first().copied());

            if let Some(out_he_id) = surviving_out_he {
                if let Some(v) = self.vertices.get_mut(start_v_id) {
                    v.outgoing_halfedge = Some(out_he_id);
                } else {
                    error!("Vertex {start_v_id:?} not found");
                }
            } else {
                self.remove_only_vertex(start_v_id);
                removed_vertices.push(start_v_id);
            }
        }

        #[cfg(feature = "instrumentation")]
        self.probe_live_face_removal(&removed_halfedges, "remove_face_tail");

        for he_id in &removed_halfedges {
            self.halfedges.remove(*he_id);
        }

        // already checked at the start of the function
        self.bvh.remove(self.faces[face_id].index);
        self.faces.remove(face_id);
        #[cfg(feature = "instrumentation")]
        crate::record_face_death(face_id);

        (removed_vertices, removed_halfedges)
    }

    /// All live halfedges whose `face` is `face_id`, sorted by key.
    ///
    /// Found locally rather than by scanning every halfedge, which made each face
    /// removal O(halfedges). The `next` chain from the face's seed halfedge finds
    /// the members of an intact face. A broken chain (twinless or dangling members,
    /// a `next` that leaves the face) can hide members from it, so the outgoing
    /// halfedges of every vertex the members touch, and their twins, are searched
    /// as well, growing the vertex set until no new member turns up. A member is
    /// missed only if it touches none of the face's vertices and the chain doesn't
    /// reach it. A seed that is dead or not a member leaves nothing to search from;
    /// only then are all halfedges scanned.
    ///
    /// Sorted by key so the result matches the slotmap-order scan this replaced:
    /// the callers free the members in this order, which decides which ids the
    /// next allocations reuse.
    fn face_members(&self, face_id: FaceId) -> Vec<HalfedgeId> {
        let mut members: Vec<HalfedgeId> = Vec::with_capacity(3);
        let mut vertices: Vec<VertexId> = Vec::with_capacity(3);

        let add =
            |he_id: HalfedgeId, members: &mut Vec<HalfedgeId>, vertices: &mut Vec<VertexId>| {
                let Some(he) = self.halfedges.get(he_id) else {
                    return false;
                };
                if he.face != Some(face_id) || members.contains(&he_id) {
                    return false;
                }
                members.push(he_id);
                push_unique(vertices, he.end_vertex);
                if let Some(twin) = he.twin.and_then(|t| self.halfedges.get(t)) {
                    push_unique(vertices, twin.end_vertex);
                }
                true
            };

        let mut he_id = self.faces.get(face_id).map(|f| f.halfedge);
        while let Some(id) = he_id {
            if !add(id, &mut members, &mut vertices) {
                break;
            }
            he_id = self.halfedges[id].next;
        }

        let scan = || {
            self.halfedges
                .iter()
                .filter(|(_, he)| he.face == Some(face_id))
                .map(|(he_id, _)| he_id)
                .collect_vec()
        };
        if members.is_empty() {
            return scan();
        }

        // `vertices` grows while it is walked.
        let mut i = 0;
        while let Some(&v_id) = vertices.get(i) {
            i += 1;
            let Some(out_he_ids) = self.outgoing_halfedges.get(v_id) else {
                continue;
            };
            for &out_he_id in out_he_ids {
                add(out_he_id, &mut members, &mut vertices);
                if let Some(twin_id) = self.halfedges.get(out_he_id).and_then(|he| he.twin) {
                    add(twin_id, &mut members, &mut vertices);
                }
            }
        }

        members.sort_unstable();

        #[cfg(feature = "instrumentation")]
        {
            let scanned = scan();
            if scanned != members {
                crate::mark_integrity_violation();
                eprintln!(
                    "FACE MEMBER SEARCH MISMATCH for face {face_id:?}: local {members:?}, scan {scanned:?}"
                );
            }
        }

        members
    }

    /// Deletes only a vertex, without deleting any faces or halfedges connected to it.
    pub fn remove_only_vertex(&mut self, vertex_id: VertexId) {
        self.outgoing_halfedges.remove(vertex_id);
        self.positions.remove(vertex_id);
        if let Some(normals) = &mut self.vertex_normals {
            normals.remove(vertex_id);
        }
        self.vertices.remove(vertex_id);
    }

    /// Deletes only a halfedge, without deleting any faces or vertices connected to it.
    /// No other halfedge is modified: the twin (if any) keeps its `twin` pointer,
    /// which now refers to a removed halfedge. The caller must re-pair the twin or
    /// remove it before its operation terminates (flap cleanup does both below).
    pub fn remove_only_halfedge(&mut self, he_id: HalfedgeId) {
        if let Some(he) = self.halfedges.get(he_id) {
            if let Some(start_v_id) = he.start_vertex(self)
                && let Some(hes) = self.outgoing_halfedges.get_mut(start_v_id)
            {
                hes.retain(|out_he_id| *out_he_id != he_id);
            }

            #[cfg(feature = "instrumentation")]
            self.probe_live_face_removal(&[he_id], "remove_only_halfedge");
            self.halfedges.remove(he_id);
        }
    }

    pub fn remove_only_halfedge_and_twin(&mut self, he_id: HalfedgeId) {
        if let Some(he) = self.halfedges.get(he_id) {
            if let Some(start_v_id) = he.start_vertex(self)
                && let Some(hes) = self.outgoing_halfedges.get_mut(start_v_id)
            {
                hes.retain(|out_he_id| *out_he_id != he_id);
            }

            if let Some(twin_he_id) = he.twin {
                self.remove_only_halfedge(twin_he_id);
            }

            // The twin was just removed. `he_id`'s own twin pointer is left as-is
            // (dead reference inside this call); nothing else survives to re-pair.
            #[cfg(feature = "instrumentation")]
            self.probe_live_face_removal(&[he_id], "remove_only_halfedge_and_twin");
            self.halfedges.remove(he_id);
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{Mat4, vec3};

    use crate::{
        utils::{build_grid, extend_with, get_tracing_subscriber},
        *,
    };

    /// A face of `mesh` none of whose edges lie on the boundary.
    fn interior_face(mesh: &MeshGraph) -> FaceId {
        mesh.faces
            .keys()
            .find(|&face_id| {
                mesh.faces[face_id].halfedges(mesh).all(|he_id| {
                    let twin_id = mesh.halfedges[he_id].twin.unwrap();
                    mesh.halfedges[twin_id].face.is_some()
                })
            })
            .unwrap()
    }

    fn has_member(mesh: &MeshGraph, face_id: FaceId) -> bool {
        mesh.halfedges.values().any(|he| he.face == Some(face_id))
    }

    /// The `next` chain stops after the seed and the last member is twinless, so
    /// the members after the seed are reachable only through the face's vertices.
    #[test]
    fn test_remove_face_broken_chain() {
        let mut mesh = build_grid(4);
        let face_id = interior_face(&mesh);
        let seed = mesh.faces[face_id].halfedge;
        let second = mesh.halfedges[seed].next.unwrap();
        let third = mesh.halfedges[second].next.unwrap();

        mesh.halfedges[seed].next = None;
        let twin = mesh.halfedges[third].twin.take().unwrap();
        mesh.halfedges[twin].twin = None;

        let (_, removed_halfedges) = mesh.remove_face(face_id);

        assert!(!mesh.faces.contains_key(face_id));
        assert!(!has_member(&mesh, face_id));
        // The twinless member is freed; the others become boundary halfedges of
        // their (interior) twins.
        assert_eq!(removed_halfedges, vec![third]);
        assert!(mesh.halfedges[seed].face.is_none());
        assert!(mesh.halfedges[second].face.is_none());
    }

    /// A seed that is not a member gives the local search nothing to start from.
    #[test]
    fn test_remove_face_foreign_seed() {
        let mut mesh = build_grid(4);
        let face_id = interior_face(&mesh);
        let other_face_id = mesh.faces.keys().find(|&f| f != face_id).unwrap();
        mesh.faces[face_id].halfedge = mesh.faces[other_face_id].halfedge;

        mesh.remove_face(face_id);

        assert!(!mesh.faces.contains_key(face_id));
        assert!(!has_member(&mesh, face_id));
        assert!(mesh.faces.contains_key(other_face_id));
    }

    #[test]
    fn test_remove_face() {
        get_tracing_subscriber();
        let mut meshgraph = MeshGraph::new();
        let p_c = vec3(0.0, 0.0, 0.0);
        let p_1 = vec3(0.0, 1.0, 0.0);
        let p_2 = vec3(-1.0, 0.5, 0.0);
        let p_3 = vec3(-1.0, -0.5, 0.0);
        let p_4 = vec3(0.0, -1.0, 0.0);
        let p_5 = vec3(1.0, -0.5, 0.0);
        let p_6 = vec3(1.0, 0.5, 0.0);

        let points = vec![p_c, p_1, p_2, p_3, p_4, p_5, p_6];
        let vertex_ids = extend_with(&mut meshgraph, &points.clone(), Mat4::default(), 5.0, 0);

        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        let face_count = meshgraph.faces.len();
        let face_id = meshgraph.vertices[vertex_ids[6]]
            .faces(&meshgraph)
            .next()
            .unwrap();
        meshgraph.remove_face(face_id);
        #[cfg(feature = "rerun")]
        {
            meshgraph.log_rerun();
            RR.flush_blocking().unwrap();
        }

        assert_eq!(meshgraph.faces.len(), face_count - 1);
    }
}
