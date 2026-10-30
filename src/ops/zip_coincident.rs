//! Zipping two sheets that lie exactly face to face.
//!
//! Position-based cleanups treat such a double layer as flaps and delete them, tearing
//! holes they can't close. Meant for welds, not for
//! [`MeshGraph::merge_vertices_one_rings`].

use glam::Vec3;
use itertools::Itertools;

use crate::hash::{HashMap, HashSet};
use crate::{FaceId, HalfedgeId, MeshGraph, VertexId};

/// What [`MeshGraph::zip_coincident_patches`] did.
#[derive(Debug, Default)]
pub struct ZipCoincident {
    pub removed_vertices: Vec<VertexId>,
    pub removed_halfedges: Vec<HalfedgeId>,
    pub removed_faces: Vec<FaceId>,

    /// `(absorbed, survivor)`. A survivor left without faces is in `removed_vertices` too.
    pub fused: Vec<(VertexId, VertexId)>,

    pub zipped_patches: usize,
    /// Found but left alone; see [`MeshGraph::zip_coincident_patches`].
    pub refused_patches: usize,
}

/// Exact position key. `+ 0.0` maps `-0.0` to `0.0`.
type PosKey = [u32; 3];

fn pos_key(p: Vec3) -> PosKey {
    [
        (p.x + 0.0).to_bits(),
        (p.y + 0.0).to_bits(),
        (p.z + 0.0).to_bits(),
    ]
}

/// A face's halfedges in `next` order; `halfedges[i]` runs `corners[i - 1] -> corners[i]`.
#[derive(Clone, Copy)]
struct Tri {
    face: FaceId,
    halfedges: [HalfedgeId; 3],
    corners: [VertexId; 3],
}

/// Coincident faces of opposite winding, indexed like `a`: `b_corner[i]` sits on
/// `a.corners[i]`, `b_halfedge[i]` lies reversed on `a.halfedges[i]`.
#[derive(Clone, Copy)]
struct FacePair {
    a: Tri,
    b: Tri,
    b_corner: [VertexId; 3],
    b_halfedge: [HalfedgeId; 3],
}

/// Two coincident halfedges just outside the patch, one per sheet, to be twinned. Starts
/// are recorded while the twins they're read from still exist.
struct Seam {
    a_out: HalfedgeId,
    b_out: HalfedgeId,
    a_out_start: VertexId,
    b_out_start: VertexId,
    /// Both are boundary: twinned they'd be a wire, so both go.
    wire: bool,
}

/// `fuse` is `(absorbed, survivor)`.
struct ZipPlan {
    seams: Vec<Seam>,
    fuse: Vec<(VertexId, VertexId)>,
}

impl MeshGraph {
    /// Zips patches where two sheets lie face to face: deletes both layers, fuses the
    /// coincident vertices along the patch's edge and twins the sheets' halfedges there.
    ///
    /// Seeds from coincident vertices in `scope`; patches grow past it, since a partly
    /// zipped patch leaves flaps at the seam. Matching is exact, like
    /// [`MeshGraph::faces_share_all_vertices`].
    ///
    /// Never opens boundary: a patch is refused if its layers touch (folds included), a
    /// vertex would fuse two ways, or a fusion would make a parallel edge. Coincident
    /// borders along the patch join and drop out.
    pub fn zip_coincident_patches(
        &mut self,
        scope: impl IntoIterator<Item = VertexId>,
    ) -> ZipCoincident {
        let mut result = ZipCoincident::default();

        // Sorted: the result must not depend on the caller's order.
        let scope = scope
            .into_iter()
            .filter(|v_id| self.vertices.contains_key(*v_id))
            .sorted()
            .dedup()
            .collect_vec();

        for patch in self.find_coincident_patches(&scope) {
            let Some(plan) = self.plan_zip(&patch) else {
                result.refused_patches += 1;
                continue;
            };

            self.apply_zip(&patch, &plan, &mut result);
            result.zipped_patches += 1;
        }

        result
    }

    /// Patches touching a coincident vertex of `scope`, fully grown, in deterministic order.
    fn find_coincident_patches(&self, scope: &[VertexId]) -> Vec<Vec<FacePair>> {
        let mut by_position: HashMap<PosKey, Vec<VertexId>> = HashMap::default();
        let mut position_order: Vec<PosKey> = Vec::new();
        for &v_id in scope {
            let Some(&position) = self.positions.get(v_id) else {
                continue;
            };
            let key = pos_key(position);
            let group = by_position.entry(key).or_default();
            if group.is_empty() {
                position_order.push(key);
            }
            group.push(v_id);
        }

        // Faces at coincident vertices, keyed by corner positions.
        let mut by_triangle: HashMap<[PosKey; 3], Vec<FaceId>> = HashMap::default();
        let mut triangle_order: Vec<[PosKey; 3]> = Vec::new();
        for key in &position_order {
            let group = &by_position[key];
            if group.len() < 2 {
                continue;
            }
            for &v_id in group {
                for face_id in self.vertex_adjacent_faces(v_id) {
                    let Some(tri) = self.tri(face_id) else {
                        continue;
                    };
                    let Some(corner_keys) = self.corner_keys(&tri) else {
                        continue;
                    };
                    let mut triangle = corner_keys;
                    triangle.sort_unstable();
                    let faces = by_triangle.entry(triangle).or_default();
                    if faces.is_empty() {
                        triangle_order.push(triangle);
                    }
                    if !faces.contains(&face_id) {
                        faces.push(face_id);
                    }
                }
            }
        }

        let mut seeds = Vec::new();
        for triangle in &triangle_order {
            // Three or more faces on one triangle aren't two sheets.
            if let [face1, face2] = by_triangle[triangle][..]
                && let Some(pair) = self.face_pair(face1, face2)
            {
                seeds.push(pair);
            }
        }

        let mut claimed: HashSet<FaceId> = HashSet::default();
        let mut patches = Vec::new();
        for seed in seeds {
            if claimed.contains(&seed.a.face) || claimed.contains(&seed.b.face) {
                continue;
            }
            claimed.insert(seed.a.face);
            claimed.insert(seed.b.face);

            // Grow across matching edges, which keeps every `a` on the seed's sheet.
            let mut patch = vec![seed];
            let mut next = 0;
            while let Some(pair) = patch.get(next).copied() {
                next += 1;
                for i in 0..3 {
                    let (Some(beyond_a), Some(beyond_b)) = (
                        self.face_beyond(pair.a.halfedges[i]),
                        self.face_beyond(pair.b_halfedge[i]),
                    ) else {
                        continue;
                    };
                    if claimed.contains(&beyond_a) || claimed.contains(&beyond_b) {
                        continue;
                    }
                    if let Some(grown) = self.face_pair(beyond_a, beyond_b) {
                        claimed.insert(beyond_a);
                        claimed.insert(beyond_b);
                        patch.push(grown);
                    }
                }
            }
            patches.push(patch);
        }

        patches
    }

    /// `None` refuses the patch.
    fn plan_zip(&self, patch: &[FacePair]) -> Option<ZipPlan> {
        let a_faces: HashSet<FaceId> = patch.iter().map(|pair| pair.a.face).collect();
        let b_faces: HashSet<FaceId> = patch.iter().map(|pair| pair.b.face).collect();
        if a_faces.len() != patch.len()
            || b_faces.len() != patch.len()
            || !a_faces.is_disjoint(&b_faces)
        {
            return None;
        }

        // One survivor per vertex; survivors are never absorbed.
        let mut fuse: Vec<(VertexId, VertexId)> = Vec::new();
        let mut survivor_of: HashMap<VertexId, VertexId> = HashMap::default();
        for pair in patch {
            for i in 0..3 {
                let (survivor, absorbed) = (pair.a.corners[i], pair.b_corner[i]);
                match survivor_of.get(&absorbed) {
                    Some(&known) if known != survivor => return None,
                    Some(_) => {}
                    None => {
                        survivor_of.insert(absorbed, survivor);
                        if absorbed != survivor {
                            fuse.push((absorbed, survivor));
                        }
                    }
                }
            }
        }
        if fuse
            .iter()
            .any(|(_, survivor)| survivor_of.get(survivor).is_some_and(|s| s != survivor))
        {
            return None;
        }

        let mut seams = Vec::new();
        let mut patch_halfedges: HashSet<HalfedgeId> = HashSet::default();
        for pair in patch {
            for i in 0..3 {
                let a_he = pair.a.halfedges[i];
                let b_he = pair.b_halfedge[i];
                patch_halfedges.insert(a_he);
                patch_halfedges.insert(b_he);

                let a_out = self.halfedges.get(a_he)?.twin?;
                let b_out = self.halfedges.get(b_he)?.twin?;
                let a_beyond = self.halfedges.get(a_out)?.face;
                let b_beyond = self.halfedges.get(b_out)?.face;

                // The layers must not touch (a fold shares this very edge).
                if a_beyond.is_some_and(|f| b_faces.contains(&f))
                    || b_beyond.is_some_and(|f| a_faces.contains(&f))
                {
                    return None;
                }

                let a_inside = a_beyond.is_some_and(|f| a_faces.contains(&f));
                let b_inside = b_beyond.is_some_and(|f| b_faces.contains(&f));
                match (a_inside, b_inside) {
                    (true, true) => {}
                    (false, false) => seams.push(Seam {
                        a_out,
                        b_out,
                        a_out_start: self.halfedges.get(a_he)?.end_vertex,
                        b_out_start: self.halfedges.get(b_he)?.end_vertex,
                        wire: a_beyond.is_none() && b_beyond.is_none(),
                    }),
                    // Interior on one side only: the holes don't match.
                    _ => return None,
                }
            }
        }

        // Surviving halfedges at fused vertices, renamed: a repeat is a parallel edge.
        let wires: HashSet<HalfedgeId> = seams
            .iter()
            .filter(|seam| seam.wire)
            .flat_map(|seam| [seam.a_out, seam.b_out])
            .collect();
        let survivor = |v_id: VertexId| survivor_of.get(&v_id).copied().unwrap_or(v_id);
        let mut directed: HashSet<(VertexId, VertexId)> = HashSet::default();
        let touched = fuse
            .iter()
            .flat_map(|&(absorbed, survivor)| [absorbed, survivor])
            .unique();
        for v_id in touched {
            for &he_id in self.outgoing_halfedges.get(v_id)? {
                if patch_halfedges.contains(&he_id) || wires.contains(&he_id) {
                    continue;
                }
                let end = self.halfedges.get(he_id)?.end_vertex;
                if !directed.insert((survivor(v_id), survivor(end))) {
                    return None;
                }
            }
        }

        Some(ZipPlan { seams, fuse })
    }

    fn apply_zip(&mut self, patch: &[FacePair], plan: &ZipPlan, result: &mut ZipCoincident) {
        let (seams, fuse) = (&plan.seams, &plan.fuse);
        // Read everything that goes through twins before rewiring any.
        let mut patch_halfedges: Vec<(HalfedgeId, VertexId)> = Vec::with_capacity(patch.len() * 6);
        for pair in patch {
            for tri in [&pair.a, &pair.b] {
                for i in 0..3 {
                    patch_halfedges.push((tri.halfedges[i], tri.corners[(i + 2) % 3]));
                }
            }
        }
        let patch_halfedge_set: HashSet<HalfedgeId> =
            patch_halfedges.iter().map(|(he_id, _)| *he_id).collect();

        let wires = seams.iter().filter(|seam| seam.wire).collect_vec();
        let wire_set: HashSet<HalfedgeId> = wires
            .iter()
            .flat_map(|seam| [seam.a_out, seam.b_out])
            .collect();

        // Halfedges ending at each absorbed vertex, to re-point.
        let mut incoming: Vec<Vec<HalfedgeId>> = Vec::with_capacity(fuse.len());
        for &(absorbed, _) in fuse {
            let ending_here = self
                .outgoing_halfedges
                .get(absorbed)
                .map(|out| {
                    out.iter()
                        .filter_map(|he_id| self.halfedges.get(*he_id)?.twin)
                        .filter(|he_id| {
                            !patch_halfedge_set.contains(he_id) && !wire_set.contains(he_id)
                        })
                        .collect_vec()
                })
                .unwrap_or_default();
            incoming.push(ending_here);
        }

        let corners = patch
            .iter()
            .flat_map(|pair| pair.a.corners.into_iter().chain(pair.b_corner))
            .unique()
            .collect_vec();

        for pair in patch {
            for face_id in [pair.a.face, pair.b.face] {
                if let Some(face) = self.faces.remove(face_id) {
                    self.bvh.remove(face.index);
                    result.removed_faces.push(face_id);
                }
            }
        }

        for seam in seams {
            self.halfedges[seam.a_out].twin = Some(seam.b_out);
            self.halfedges[seam.b_out].twin = Some(seam.a_out);
        }

        for &(he_id, start) in &patch_halfedges {
            self.drop_halfedge(he_id, start, result);
        }

        for seam in &wires {
            self.drop_halfedge(seam.a_out, seam.a_out_start, result);
            self.drop_halfedge(seam.b_out, seam.b_out_start, result);
        }

        for (&(absorbed, survivor), ending_here) in fuse.iter().zip(incoming) {
            for he_id in ending_here {
                if let Some(he) = self.halfedges.get_mut(he_id) {
                    he.end_vertex = survivor;
                }
            }
            let outgoing = self
                .outgoing_halfedges
                .get(absorbed)
                .cloned()
                .unwrap_or_default();
            if let Some(entry) = self.outgoing_halfedges.entry(survivor) {
                entry.or_default().extend(outgoing);
            }
            self.remove_only_vertex(absorbed);
            result.removed_vertices.push(absorbed);
            result.fused.push((absorbed, survivor));
        }

        for v_id in corners {
            if !self.vertices.contains_key(v_id) {
                continue;
            }
            if self
                .outgoing_halfedges
                .get(v_id)
                .is_none_or(|out| out.is_empty())
            {
                self.remove_only_vertex(v_id);
                result.removed_vertices.push(v_id);
                continue;
            }
            self.reseed_outgoing_if_dead(v_id);
            self.make_outgoing_halfedge_boundary_if_possible(v_id);
            self.compute_vertex_normal(v_id);
        }
    }

    /// `start` is passed in: the twin it'd be read from may be gone.
    fn drop_halfedge(&mut self, he_id: HalfedgeId, start: VertexId, result: &mut ZipCoincident) {
        if let Some(out) = self.outgoing_halfedges.get_mut(start) {
            out.retain(|id| *id != he_id);
        }
        if self.halfedges.remove(he_id).is_some() {
            result.removed_halfedges.push(he_id);
        }
    }

    fn face_beyond(&self, he_id: HalfedgeId) -> Option<FaceId> {
        let twin = self.halfedges.get(he_id)?.twin?;
        self.halfedges.get(twin)?.face
    }

    fn tri(&self, face_id: FaceId) -> Option<Tri> {
        let face = self.faces.get(face_id)?;
        let halfedges: [HalfedgeId; 3] = face.halfedges(self).collect_vec().try_into().ok()?;
        let mut corners = [VertexId::default(); 3];
        for i in 0..3 {
            let he = self.halfedges.get(halfedges[i])?;
            if he.face != Some(face_id) {
                return None;
            }
            corners[i] = he.end_vertex;
        }
        Some(Tri {
            face: face_id,
            halfedges,
            corners,
        })
    }

    fn corner_keys(&self, tri: &Tri) -> Option<[PosKey; 3]> {
        let [p0, p1, p2] = tri.corners.map(|v_id| self.positions.get(v_id).copied());
        Some([pos_key(p0?), pos_key(p1?), pos_key(p2?)])
    }

    /// A pair if the corners coincide with opposite winding. Degenerate faces and faces on
    /// the same three ids don't count.
    fn face_pair(&self, face1: FaceId, face2: FaceId) -> Option<FacePair> {
        if face1 == face2 {
            return None;
        }
        let (a, b) = (self.tri(face1)?, self.tri(face2)?);
        let (a_keys, b_keys) = (self.corner_keys(&a)?, self.corner_keys(&b)?);
        if a_keys[0] == a_keys[1] || a_keys[1] == a_keys[2] || a_keys[0] == a_keys[2] {
            return None;
        }

        let rotation = (0..3).find(|&r| (0..3).all(|i| b_keys[(r + 3 - i) % 3] == a_keys[i]))?;
        let b_index = |i: usize| (rotation + 3 - i) % 3;

        let b_corner = [0, 1, 2].map(|i| b.corners[b_index(i)]);
        if b_corner == a.corners {
            return None;
        }
        // The halfedge of `b` on `a[i-1] -> a[i]` ends at `b[i-1]`.
        let b_halfedge = [0, 1, 2].map(|i| b.halfedges[b_index((i + 2) % 3)]);

        Some(FacePair {
            a,
            b,
            b_corner,
            b_halfedge,
        })
    }
}

#[cfg(test)]
mod tests {
    use glam::vec3;

    use super::*;

    /// Two meshes in one, sharing no vertices.
    fn build(
        points_a: &[Vec3],
        faces_a: &[[usize; 3]],
        points_b: &[Vec3],
        faces_b: &[[usize; 3]],
    ) -> (MeshGraph, Vec<VertexId>) {
        let mut mesh = MeshGraph::new();
        let mut v_ids = Vec::new();
        for (points, faces) in [(points_a, faces_a), (points_b, faces_b)] {
            let ids = points.iter().map(|p| mesh.add_vertex(*p)).collect_vec();
            for [i, j, k] in faces {
                mesh.add_face_from_vertices(ids[*i], ids[*j], ids[*k])
                    .expect("test face");
            }
            v_ids.extend(ids);
        }
        (mesh, v_ids)
    }

    fn boundary_edges(mesh: &MeshGraph) -> std::collections::BTreeSet<(VertexId, VertexId)> {
        mesh.halfedges
            .values()
            .filter(|he| he.face.is_none())
            .filter_map(|he| {
                let start = he.start_vertex(mesh)?;
                Some((start.min(he.end_vertex), start.max(he.end_vertex)))
            })
            .collect()
    }

    fn assert_closed_manifold(mesh: &MeshGraph) {
        for (he_id, he) in mesh.halfedges.iter() {
            assert!(he.face.is_some(), "boundary halfedge {he_id:?}");
        }
        assert_manifold(mesh);
    }

    /// Twins agree, no wires, outgoing lists right with live seeds, no parallel edges, no
    /// isolated vertices.
    fn assert_manifold(mesh: &MeshGraph) {
        for (he_id, he) in mesh.halfedges.iter() {
            let twin = he.twin.expect("twin");
            assert_eq!(mesh.halfedges[twin].twin, Some(he_id), "twins disagree");
            assert!(
                he.face.is_some() || mesh.halfedges[twin].face.is_some(),
                "wire edge at {he_id:?}"
            );
        }
        for (v_id, out) in mesh.outgoing_halfedges.iter() {
            assert!(!out.is_empty(), "isolated vertex {v_id:?}");
            let seed = mesh.vertices[v_id].outgoing_halfedge.expect("seed");
            assert!(out.contains(&seed), "dead or foreign seed at {v_id:?}");
            let mut ends = Vec::new();
            for he_id in out {
                let he = mesh.halfedges.get(*he_id).expect("live outgoing halfedge");
                assert_eq!(
                    he.start_vertex(mesh),
                    Some(v_id),
                    "outgoing list of {v_id:?}"
                );
                assert!(!ends.contains(&he.end_vertex), "parallel edge at {v_id:?}");
                ends.push(he.end_vertex);
            }
        }
    }

    fn square_pyramids(b_diagonal: [[usize; 3]; 2]) -> (MeshGraph, Vec<VertexId>) {
        let base = [
            vec3(0., 0., 0.),
            vec3(1., 0., 0.),
            vec3(1., 1., 0.),
            vec3(0., 1., 0.),
        ];
        let mut up = base.to_vec();
        up.push(vec3(0.5, 0.5, 1.));
        let mut down = base.to_vec();
        down.push(vec3(0.5, 0.5, -1.));
        let up_faces = [
            [0, 2, 1],
            [0, 3, 2],
            [0, 1, 4],
            [1, 2, 4],
            [2, 3, 4],
            [3, 0, 4],
        ];
        let mut down_faces = vec![[1, 0, 4], [2, 1, 4], [3, 2, 4], [0, 3, 4]];
        down_faces.extend(b_diagonal);
        build(&up, &up_faces, &down, &down_faces)
    }

    #[test]
    fn test_zips_two_tetrahedra_into_a_bipyramid() {
        let base = [vec3(0., 0., 0.), vec3(1., 0., 0.), vec3(0., 1., 0.)];
        let mut up = base.to_vec();
        up.push(vec3(0.3, 0.3, 1.));
        let mut down = base.to_vec();
        down.push(vec3(0.3, 0.3, -1.));
        let (mut mesh, v_ids) = build(
            &up,
            &[[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
            &down,
            &[[0, 1, 2], [1, 0, 3], [2, 1, 3], [0, 2, 3]],
        );

        let zip = mesh.zip_coincident_patches(v_ids.iter().copied());

        assert_eq!((zip.zipped_patches, zip.refused_patches), (1, 0));
        assert_eq!(zip.removed_faces.len(), 2);
        assert_eq!(zip.fused.len(), 3);
        assert_eq!((mesh.vertices.len(), mesh.faces.len()), (5, 6));
        assert_closed_manifold(&mesh);
    }

    #[test]
    fn test_zips_a_two_face_patch_and_drops_its_inner_edge() {
        let (mut mesh, v_ids) = square_pyramids([[0, 1, 2], [0, 2, 3]]);

        let zip = mesh.zip_coincident_patches(v_ids.iter().copied());

        assert_eq!((zip.zipped_patches, zip.refused_patches), (1, 0));
        assert_eq!(zip.removed_faces.len(), 4);
        assert_eq!(zip.fused.len(), 4);
        // An octahedron.
        assert_eq!((mesh.vertices.len(), mesh.faces.len()), (6, 8));
        assert_closed_manifold(&mesh);
    }

    #[test]
    fn test_grows_the_patch_past_the_scope() {
        let (mut mesh, v_ids) = square_pyramids([[0, 1, 2], [0, 2, 3]]);

        // One base corner and its partner.
        let zip = mesh.zip_coincident_patches([v_ids[1], v_ids[6]]);

        assert_eq!(zip.zipped_patches, 1);
        assert_eq!(zip.removed_faces.len(), 4);
        assert_closed_manifold(&mesh);
    }

    #[test]
    fn test_leaves_differently_tessellated_contact_alone() {
        let (mut mesh, v_ids) = square_pyramids([[1, 2, 3], [1, 3, 0]]);
        let (vertices, faces) = (mesh.vertices.len(), mesh.faces.len());

        let zip = mesh.zip_coincident_patches(v_ids.iter().copied());

        assert_eq!((zip.zipped_patches, zip.refused_patches), (0, 0));
        assert_eq!((mesh.vertices.len(), mesh.faces.len()), (vertices, faces));
        assert_closed_manifold(&mesh);
    }

    /// `(s0, v, s1)` and `(s0, s1, w)` share `s0-s1`, with `v` on `w`.
    #[test]
    fn test_leaves_a_fold_alone() {
        // s0, s1, v, w, p, q
        let points = [
            vec3(0., 0., 0.),
            vec3(1., 0., 0.),
            vec3(0.5, 1., 0.),
            vec3(0.5, 1., 0.),
            vec3(0.5, 0.5, 1.),
            vec3(0.5, 0.5, -1.),
        ];
        let faces = [
            [0, 2, 1],
            [0, 1, 3],
            [2, 0, 4],
            [1, 2, 4],
            [3, 1, 5],
            [0, 3, 5],
            [4, 0, 5],
            [5, 1, 4],
        ];
        let (mut mesh, v_ids) = build(&points, &faces, &[], &[]);
        assert_closed_manifold(&mesh);
        let (vertices, faces) = (mesh.vertices.len(), mesh.faces.len());

        let zip = mesh.zip_coincident_patches(v_ids.iter().copied());

        assert_eq!((zip.zipped_patches, zip.refused_patches), (0, 1));
        assert_eq!((mesh.vertices.len(), mesh.faces.len()), (vertices, faces));
        assert_closed_manifold(&mesh);
    }

    /// Both corner 0s reach `x`, so fusing them would join it twice.
    #[test]
    fn test_refuses_a_zip_that_would_make_a_parallel_edge() {
        // a0, a1, a2, b0, b1, b2, x, y
        let points = [
            vec3(0., 0., 0.),
            vec3(1., 0., 0.),
            vec3(0., 1., 0.),
            vec3(0., 0., 0.),
            vec3(1., 0., 0.),
            vec3(0., 1., 0.),
            vec3(0.5, -1., 0.5),
            vec3(0.5, -1., -0.5),
        ];
        let faces = [[0, 1, 2], [1, 0, 6], [3, 5, 4], [3, 4, 7], [3, 7, 6]];
        let (mut mesh, v_ids) = build(&points, &faces, &[], &[]);
        let (vertices, faces, boundary) =
            (mesh.vertices.len(), mesh.faces.len(), boundary_edges(&mesh));

        let zip = mesh.zip_coincident_patches(v_ids.iter().copied());

        assert_eq!((zip.zipped_patches, zip.refused_patches), (0, 1));
        assert_eq!((mesh.vertices.len(), mesh.faces.len()), (vertices, faces));
        assert_eq!(boundary_edges(&mesh), boundary);
    }

    /// Open sheets: the coincident borders `1-2`, `2-0` drop out, the own faces join at
    /// `0-1`.
    #[test]
    fn test_coincident_borders_join_and_drop_out() {
        let base = [vec3(0., 0., 0.), vec3(1., 0., 0.), vec3(0., 1., 0.)];
        let mut a = base.to_vec();
        a.push(vec3(0.5, -1., 0.5));
        let mut b = base.to_vec();
        b.push(vec3(0.5, -1., -0.5));
        let (mut mesh, v_ids) = build(&a, &[[0, 1, 2], [1, 0, 3]], &b, &[[0, 2, 1], [0, 1, 3]]);
        assert_eq!(boundary_edges(&mesh).len(), 8);

        let zip = mesh.zip_coincident_patches(v_ids.iter().copied());

        assert_eq!((zip.zipped_patches, zip.refused_patches), (1, 0));
        assert_eq!(zip.removed_faces.len(), 2);
        let mut fused = zip.fused.clone();
        fused.sort();
        assert_eq!(
            fused,
            vec![
                (v_ids[4], v_ids[0]),
                (v_ids[5], v_ids[1]),
                (v_ids[6], v_ids[2])
            ]
        );
        assert!(!mesh.vertices.contains_key(v_ids[2]));

        let edge = |i: usize, j: usize| (v_ids[i].min(v_ids[j]), v_ids[i].max(v_ids[j]));
        assert_eq!(
            boundary_edges(&mesh),
            [edge(0, 3), edge(1, 3), edge(0, 7), edge(1, 7)]
                .into_iter()
                .collect()
        );
        assert_eq!((mesh.vertices.len(), mesh.faces.len()), (4, 2));
        assert_manifold(&mesh);
    }

    #[test]
    fn test_result_does_not_depend_on_scope_order() {
        let (mut forward, v_ids) = square_pyramids([[0, 1, 2], [0, 2, 3]]);
        let (mut backward, _) = square_pyramids([[0, 1, 2], [0, 2, 3]]);

        let zip_forward = forward.zip_coincident_patches(v_ids.iter().copied());
        let zip_backward = backward.zip_coincident_patches(v_ids.iter().rev().copied());

        assert_eq!(zip_forward.removed_faces, zip_backward.removed_faces);
        assert_eq!(
            zip_forward.removed_halfedges,
            zip_backward.removed_halfedges
        );
        assert_eq!(zip_forward.fused, zip_backward.fused);
    }
}
