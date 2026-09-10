use crate::{
    utils::{extend_with, get_tracing_subscriber},
    *,
};
use glam::*;
use hashbrown::HashSet;

#[test]
fn test_vertex_merge_equal_count() {
    get_tracing_subscriber();

    let mut meshgraph = MeshGraph::new();
    let p_c = vec3(0.0, 0.0, 1.0);
    let p_1 = vec3(0.0, 1.0, 0.0);
    let p_2 = vec3(-1.0, 0.5, 0.0);
    let p_3 = vec3(-1.0, -0.5, 0.0);
    let p_4 = vec3(0.0, -1.0, 0.0);
    let p_5 = vec3(1.0, -0.5, 0.0);
    let p_6 = vec3(1.0, 0.5, 0.0);

    let points = vec![p_c, p_1, p_2, p_3, p_4, p_5, p_6];
    let v_c_id = extend_with(&mut meshgraph, &points.clone(), Mat4::default(), 2.0, 1)[0];

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    // duplicates a mirrored version of the mesh above
    let mirror_mat = Mat4::from_rotation_translation(
        Quat::from_rotation_x(std::f32::consts::PI)
            .mul_quat(Quat::from_rotation_z(std::f32::consts::PI * 0.5)),
        vec3(0.0, 0.0, 3.0),
    );

    let v_c_m_id = extend_with(&mut meshgraph, &points, mirror_mat, 2.0, 1)[0];

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_c_id,
        v_c_m_id,
        1.0,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 12);
    assert_eq!(result.removed_halfedges.len(), 24);
    assert_eq!(result.removed_vertices.len(), 2);

    assert_eq!(result.added_faces.len(), 12);
    assert_eq!(result.added_halfedges.len(), 24);
    assert_eq!(result.added_vertices.len(), 0);

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), 0);
}

#[test]
fn test_vertex_merge_different_count() {
    let mut meshgraph = MeshGraph::new();
    let p_c = vec3(0.0, 0.0, 1.0);
    let p_1 = vec3(0.0, 1.0, 0.0);
    let p_2 = vec3(-1.0, 0.5, 0.0);
    let p_3 = vec3(-1.0, -0.5, 0.0);
    let p_4 = vec3(0.0, -1.0, 0.0);
    let p_5 = vec3(1.0, -0.5, 0.0);
    let p_6 = vec3(1.0, 0.5, 0.0);

    let v_c_id = extend_with(
        &mut meshgraph,
        &[p_c, p_1, p_2, p_3, p_4, p_5, p_6],
        Mat4::default(),
        2.0,
        1,
    )[0];

    let mirror_mat = Mat4::from_rotation_translation(
        Quat::from_rotation_x(std::f32::consts::PI)
            .mul_quat(Quat::from_rotation_z(std::f32::consts::PI * 0.5)),
        vec3(0.0, 0.0, 3.0),
    );

    let p_1 = vec3(0.0, 1.0, 0.0);
    let p_2 = vec3(-1.0, 0.0, 0.0);
    let p_3 = vec3(-0.5, -1.0, 0.0);
    let p_4 = vec3(0.5, -1.0, 0.0);
    let p_5 = vec3(1.0, 0.0, 0.0);

    let v_c_m_id = extend_with(
        &mut meshgraph,
        &[p_c, p_1, p_2, p_3, p_4, p_5],
        mirror_mat,
        2.0,
        1,
    )[0];

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_c_id,
        v_c_m_id,
        1.0,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 11);
    assert_eq!(result.removed_halfedges.len(), 22);
    assert_eq!(result.removed_vertices.len(), 2);

    assert_eq!(result.added_faces.len(), 11);
    assert_eq!(result.added_halfedges.len(), 22);

    assert_eq!(marked_halfedges.len(), 22);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_one_ring() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_common_one_ring.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        1.0,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 21);
    assert_eq!(result.removed_halfedges.len(), 46);
    assert_eq!(result.removed_vertices.len(), 3);

    assert_eq!(result.added_faces.len(), 13);
    assert_eq!(result.added_halfedges.len(), 22);
    assert_eq!(result.added_vertices.len(), 1);

    assert_eq!(marked_halfedges.len(), 22);
    assert_eq!(marked_vertices.len(), 2);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_except_one() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_except_one.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        1.0,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 10);
    assert_eq!(result.removed_halfedges.len(), 26);
    assert_eq!(result.removed_vertices.len(), 4);

    assert_eq!(result.added_faces.len(), 2);
    assert_eq!(result.added_halfedges.len(), 2);
    assert_eq!(result.added_vertices.len(), 0);

    assert_eq!(marked_halfedges.len(), 2);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_except_two() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_except_one_two.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        1.0,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 11);
    assert_eq!(result.removed_halfedges.len(), 28);
    assert_eq!(result.removed_vertices.len(), 4);

    assert_eq!(result.added_faces.len(), 3);
    assert_eq!(result.added_halfedges.len(), 4);

    assert_eq!(marked_halfedges.len(), 4);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_single_he_flip() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_single_he.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        1.0,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 2);
    assert_eq!(result.removed_halfedges.len(), 6);
    assert_eq!(result.removed_vertices.len(), 1);

    assert_eq!(result.added_faces.len(), 0);
    assert_eq!(result.added_halfedges.len(), 0);

    assert_eq!(marked_halfedges.len(), 0);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_single_he_noflip() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_single_he.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 12);
    assert_eq!(result.removed_halfedges.len(), 26);
    assert_eq!(result.removed_vertices.len(), 2);

    assert_eq!(result.added_faces.len(), 8);
    assert_eq!(result.added_halfedges.len(), 14);

    assert_eq!(marked_halfedges.len(), 14);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_two_one_separate() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_two_one_separate.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.y == 0.0 {
            if pos.x == 0.0 {
                v_top_id = v_id;
            } else if pos.x == 1.0 {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 10);
    assert_eq!(result.removed_halfedges.len(), 24);
    assert_eq!(result.removed_vertices.len(), 3);

    assert_eq!(result.added_faces.len(), 4);
    assert_eq!(result.added_halfedges.len(), 6);

    assert_eq!(marked_halfedges.len(), 6);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_one_vert() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_common_one_vert.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 10);
    assert_eq!(result.removed_halfedges.len(), 20);
    assert_eq!(result.removed_vertices.len(), 2);

    assert_eq!(result.added_faces.len(), 8);
    assert_eq!(result.added_halfedges.len(), 14);

    assert_eq!(marked_halfedges.len(), 14);
    assert_eq!(marked_vertices.len(), 2);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_double_flap() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_double_flap.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == -1.0 && pos.y == -1.0 {
            v_top_id = v_id;
        } else if pos.x == 1.0 && pos.y == 1.0 {
            v_bottom_id = v_id;
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    // `find_already_connected_pairings` only takes the first strip of existing
    // faces now (a second strip would otherwise leave the region between the
    // strips uncovered), so the connect step seams this mesh one edge
    // differently: 23 (was 22) removed, 5 (was 4) added — same faces in/out
    // (9/3), same net halfedge delta, boundary unchanged, all integrity probes
    // clean.
    assert_eq!(result.removed_faces.len(), 9, "REMOVED_FACES");
    assert_eq!(result.removed_halfedges.len(), 23, "REMOVED_HALFEDGES");
    assert_eq!(result.removed_vertices.len(), 3, "REMOVED_VERTICES");

    assert_eq!(result.added_faces.len(), 3, "ADDED_FACES");
    assert_eq!(result.added_halfedges.len(), 5, "ADDED_HALFEDGES");
    assert_eq!(result.added_vertices.len(), 0, "ADDED_VERTICES");

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_two_5_8() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_common_two_5_8.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == -1.0 && pos.y == -1.0 {
            v_top_id = v_id;
        } else if pos.x == 1.0 && pos.y == 1.0 {
            v_bottom_id = v_id;
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 13);
    assert_eq!(result.removed_halfedges.len(), 30);
    assert_eq!(result.removed_vertices.len(), 3);

    assert_eq!(result.added_faces.len(), 7);
    assert_eq!(result.added_halfedges.len(), 12);

    assert_eq!(marked_halfedges.len(), 12);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_one_4_9() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_common_one_4_9.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == -1.0 && pos.y == -1.0 {
            v_top_id = v_id;
        } else if pos.x == 1.0 && pos.y == 1.0 {
            v_bottom_id = v_id;
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 16);
    assert_eq!(result.removed_halfedges.len(), 38);
    assert_eq!(result.removed_vertices.len(), 4);

    assert_eq!(result.added_faces.len(), 8);
    assert_eq!(result.added_halfedges.len(), 14);
    assert_eq!(result.added_vertices.len(), 0);

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_connected_hes() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_connected_hes.glb").unwrap();
    meshgraph.compute_vertex_normals();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 15);
    assert_eq!(result.removed_halfedges.len(), 36);
    assert_eq!(result.removed_vertices.len(), 4);

    assert_eq!(result.added_faces.len(), 7);
    assert_eq!(result.added_halfedges.len(), 12);
    assert_eq!(result.added_vertices.len(), 0);

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_two_vs_3_4() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_two_vs_3_4.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 8);
    assert_eq!(result.removed_halfedges.len(), 20);

    assert_eq!(result.added_faces.len(), 2);
    assert_eq!(result.added_halfedges.len(), 2);

    assert_eq!(marked_halfedges.len(), 2);
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_two_consec_vs_one_he_9_10() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_two_consec_vs_one_he_9_10.glb")
            .unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 21);
    assert_eq!(result.removed_halfedges.len(), 48);
    assert_eq!(result.removed_vertices.len(), 3);

    assert_eq!(result.added_faces.len(), 11);
    assert_eq!(result.added_halfedges.len(), 18);
    assert_eq!(result.added_vertices.len(), 0);

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_two_hes_hole() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_two_hes_hole.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == -1.0 && pos.y == -1.0 {
            v_top_id = v_id;
        } else if pos.x == 1.0 && pos.y == 1.0 {
            v_bottom_id = v_id;
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 18);
    assert_eq!(result.removed_halfedges.len(), 40);
    assert_eq!(result.removed_vertices.len(), 2);

    assert_eq!(result.added_faces.len(), 10);
    assert_eq!(result.added_halfedges.len(), 16);

    assert_eq!(marked_halfedges.len(), 16);
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_common_vert_he_tube() {
    use crate::integrations::gltf;

    get_tracing_subscriber();
    let mut meshgraph =
        gltf::load("src/ops/merge_one_ring/glb/merge_common_vert_he_tube.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 13);
    assert_eq!(result.removed_halfedges.len(), 28);
    assert_eq!(result.removed_vertices.len(), 2);

    assert_eq!(result.added_faces.len(), 7);
    assert_eq!(result.added_halfedges.len(), 10);
    assert_eq!(result.added_vertices.len(), 1);

    assert_eq!(marked_halfedges.len(), 10);
    assert_eq!(marked_vertices.len(), 2);
}

#[cfg(feature = "gltf")]
#[test]
fn test_merge_4_4() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_4_4.glb").unwrap();
    meshgraph.compute_vertex_normals();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == -1.0 && pos.y == -1.0 {
            v_top_id = v_id;
        } else if pos.x == 1.0 && pos.y == 1.0 {
            v_bottom_id = v_id;
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    #[cfg(feature = "rerun")]
    meshgraph.log_verts_rerun("merge", &[v_top_id, v_bottom_id]);

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        use itertools::Itertools;

        meshgraph.log_rerun();
        meshgraph.log_verts_rerun("marked", &marked_vertices.iter().copied().collect_vec());
        RR.flush_blocking().unwrap();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 11);
    assert_eq!(result.removed_halfedges.len(), 26);
    assert_eq!(result.removed_vertices.len(), 3);

    assert_eq!(result.added_faces.len(), 5);
    assert_eq!(result.added_halfedges.len(), 8);
    assert_eq!(result.added_vertices.len(), 0);

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_6_6_tip() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_6_6_tip.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert!(
        result.removed_faces.len() >= 15,
        "removed_faces.len() = {}",
        result.removed_faces.len()
    );
    assert!(
        result.removed_halfedges.len() >= 36,
        "removed_halfedges.len() = {}",
        result.removed_halfedges.len()
    );
    assert!(
        result.removed_vertices.len() >= 4,
        "removed_vertices.len() = {}",
        result.removed_vertices.len()
    );

    assert_eq!(result.added_faces.len(), 4);
    assert_eq!(result.added_halfedges.len(), 6);
    assert!(
        result.added_vertices.len() <= 1,
        "added_vertices.len() = {}",
        result.added_vertices.len()
    );

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), 0);
}

#[cfg(feature = "gltf")]
#[test]
fn test_vertex_merge_6_6_tube() {
    use crate::integrations::gltf;

    get_tracing_subscriber();

    let mut meshgraph = gltf::load("src/ops/merge_one_ring/glb/merge_6_6_tube.glb").unwrap();

    #[cfg(feature = "rerun")]
    meshgraph.log_rerun();

    let mut v_top_id = VertexId::default();
    let mut v_bottom_id = VertexId::default();

    for (v_id, pos) in &meshgraph.positions {
        if pos.x == 0.0 && pos.y == 0.0 {
            if pos.z > 0.0 {
                v_top_id = v_id;
            } else {
                v_bottom_id = v_id;
            }
        }
    }

    if v_top_id == VertexId::default() {
        panic!("No top vertex found");
    }

    if v_bottom_id == VertexId::default() {
        panic!("No bottom vertex found");
    }

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();

    let result = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    assert_eq!(result.removed_faces.len(), 12,);
    assert_eq!(result.removed_halfedges.len(), 26,);
    assert_eq!(result.removed_vertices.len(), 2,);

    assert_eq!(result.added_faces.len(), 8,);
    assert_eq!(result.added_halfedges.len(), 14,);
    assert_eq!(result.added_vertices.len(), 2,);

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), 4);
}

/// Regression test for the weld-hole defect: `merge_vertices_one_rings` used to
/// end its op with boundary halfedges (face = None) — holes in a mesh that must
/// stay closed.
///
/// The region below was captured with a one-off region dump (no longer in the
/// tree) from the `weld_hole_dump` snapshot replay
/// (freestyle-sculpt `snapshot_replay_weld_hole`): the exact state around
/// vertices 1487 and 1490 right before their one-rings are merged — the two
/// vertices, both one-rings and every face incident to either of them (the
/// full star of the merged region; 40 vertices / 57 faces, all triangles, no
/// parallel halfedge pairs).
///
/// The test rebuilds that region and replays the single merge
/// `merge_vertices_one_rings(1487, 1490)`. A watertight merge must not create
/// boundary halfedges: the boundary edges after the op must be exactly the
/// region's pre-existing outer cut against the rest of the mesh. The merge used
/// to detach leftover strip halfedges (the weld hole); this test guards the fix.
#[test]
fn test_merge_one_ring_weld_hole_reproduction() {
    get_tracing_subscriber();

    // vertex id -> rebuild id mapping is the sorted array index (see comments).
    const REGION_VERTICES: [(f32, f32, f32); 40] = [
        (-19.356205, -41.10125, 20.097294),  // 1303 -> 0
        (-19.370916, -40.972824, 20.200298), // 1304 -> 1
        (-19.20674, -40.943718, 20.240244),  // 1305 -> 2
        (-19.348406, -40.905327, 20.09614),  // 1441 -> 3
        (-19.280746, -40.655098, 20.177124), // 1451 -> 4
        (-19.259031, -40.45329, 19.841354),  // 1452 -> 5
        (-19.12634, -40.58493, 20.132698),   // 1453 -> 6
        (-19.104523, -40.247227, 19.746563), // 1465 -> 7
        (-19.001717, -40.262856, 20.026884), // 1466 -> 8
        (-18.951622, -40.389786, 20.268528), // 1467 -> 9
        (-19.563955, -40.65847, 20.699993),  // 1470 -> 10
        (-19.631655, -40.62883, 21.004108),  // 1471 -> 11
        (-19.533033, -40.79623, 20.39446),   // 1473 -> 12
        (-19.503408, -40.520206, 21.156343), // 1474 -> 13
        (-19.389235, -40.519855, 20.923641), // 1475 -> 14
        (-19.448322, -40.478523, 21.424534), // 1480 -> 15
        (-19.344837, -40.626804, 20.63328),  // 1486 -> 16
        (-19.093132, -40.493454, 20.782125), // 1487 -> 17
        (-18.991308, -40.72907, 20.360819),  // 1490 -> 18
        (-19.157166, -40.862274, 20.260632), // 1491 -> 19
        (-18.942017, -40.78793, 20.451096),  // 1492 -> 20
        (-18.868355, -40.76729, 20.50477),   // 1493 -> 21
        (-19.05429, -40.710136, 20.434969),  // 1494 -> 22
        (-19.237757, -40.41564, 21.113014),  // 1495 -> 23
        (-19.229782, -40.371414, 21.38882),  // 1496 -> 24
        (-18.979753, -40.28127, 21.311527),  // 1497 -> 25
        (-18.925571, -40.367558, 20.967243), // 1498 -> 26
        (-18.801954, -40.171688, 20.179123), // 1510 -> 27
        (-18.813383, -40.724464, 20.539238), // 1516 -> 28
        (-18.782833, -40.708508, 20.554337), // 1517 -> 29
        (-18.806547, -40.549667, 20.508982), // 1518 -> 30
        (-18.742113, -40.507896, 20.731491), // 1519 -> 31
        (-18.64833, -40.666275, 20.626318),  // 1520 -> 32
        (-18.628391, -40.6607, 20.636421),   // 1521 -> 33
        (-18.55072, -40.62165, 20.67177),    // 1522 -> 34
        (-18.454082, -40.592327, 20.702282), // 1523 -> 35
        (-18.419567, -40.545345, 20.738708), // 1526 -> 36
        (-18.744991, -40.293556, 20.415895), // 1527 -> 37
        (-18.641088, -40.315002, 20.978395), // 1529 -> 38
        (-18.538479, -40.410683, 20.630009), // 1530 -> 39
    ];
    const REGION_FACES: [(usize, usize, usize); 57] = [
        (6, 19, 18),  // (1453, 1491, 1490)
        (3, 22, 1),   // (1441, 1494, 1304)
        (22, 3, 6),   // (1494, 1441, 1453)
        (16, 10, 11), // (1486, 1470, 1471)
        (16, 12, 10), // (1486, 1473, 1470)
        (14, 11, 13), // (1475, 1471, 1474)
        (16, 11, 14), // (1486, 1471, 1475)
        (23, 14, 13), // (1495, 1475, 1474)
        (15, 23, 13), // (1480, 1495, 1474)
        (16, 4, 12),  // (1486, 1451, 1473)
        (22, 4, 16),  // (1494, 1451, 1486)
        (4, 22, 6),   // (1451, 1494, 1453)
        (16, 14, 17), // (1486, 1475, 1487)
        (17, 22, 16), // (1487, 1494, 1486)
        (17, 14, 23), // (1487, 1475, 1495)
        (24, 23, 15), // (1496, 1495, 1480)
        (26, 17, 23), // (1498, 1487, 1495)
        (19, 22, 20), // (1491, 1494, 1492)
        (20, 22, 21), // (1492, 1494, 1493)
        (21, 22, 28), // (1493, 1494, 1516)
        (28, 22, 29), // (1516, 1494, 1517)
        (31, 22, 17), // (1519, 1494, 1487)
        (31, 17, 26), // (1519, 1487, 1498)
        (25, 23, 24), // (1497, 1495, 1496)
        (26, 23, 25), // (1498, 1495, 1497)
        (38, 26, 25), // (1529, 1498, 1497)
        (29, 22, 31), // (1517, 1494, 1519)
        (29, 31, 32), // (1517, 1519, 1520)
        (32, 31, 33), // (1520, 1519, 1521)
        (33, 31, 34), // (1521, 1519, 1522)
        (36, 31, 38), // (1526, 1519, 1529)
        (34, 31, 35), // (1522, 1519, 1523)
        (35, 31, 36), // (1523, 1519, 1526)
        (31, 26, 38), // (1519, 1498, 1529)
        (19, 3, 0),   // (1491, 1441, 1303)
        (0, 2, 19),   // (1303, 1305, 1491)
        (19, 6, 3),   // (1491, 1453, 1441)
        (6, 5, 4),    // (1453, 1452, 1451)
        (6, 18, 9),   // (1453, 1490, 1467)
        (7, 5, 6),    // (1465, 1452, 1453)
        (8, 7, 6),    // (1466, 1465, 1453)
        (8, 6, 9),    // (1466, 1453, 1467)
        (27, 9, 37),  // (1510, 1467, 1527)
        (8, 9, 27),   // (1466, 1467, 1510)
        (1, 19, 2),   // (1304, 1491, 1305)
        (19, 1, 22),  // (1491, 1304, 1494)
        (20, 18, 19), // (1492, 1490, 1491)
        (21, 18, 20), // (1493, 1490, 1492)
        (28, 18, 21), // (1516, 1490, 1493)
        (9, 18, 30),  // (1467, 1490, 1518)
        (30, 18, 28), // (1518, 1490, 1516)
        (37, 9, 30),  // (1527, 1467, 1518)
        (29, 30, 28), // (1517, 1518, 1516)
        (32, 30, 29), // (1520, 1518, 1517)
        (33, 30, 32), // (1521, 1518, 1520)
        (37, 30, 39), // (1527, 1518, 1530)
        (39, 30, 33), // (1530, 1518, 1521)
    ];
    // The two vertices whose one-rings are merged (1487 -> 17, 1490 -> 18).
    const MERGE_V1: usize = 17;
    const MERGE_V2: usize = 18;
    // flip threshold of the weld input log (log 010_weld).
    const FLIP_THRESHOLD_SQR: f32 = 2.1599998;

    let mut mesh_graph = MeshGraph::new();

    let mut vertex_ids = Vec::with_capacity(REGION_VERTICES.len());
    for &(x, y, z) in &REGION_VERTICES {
        vertex_ids.push(mesh_graph.add_vertex(vec3(x, y, z)));
    }
    for &(a, b, c) in &REGION_FACES {
        mesh_graph
            .add_face_from_vertices(vertex_ids[a], vertex_ids[b], vertex_ids[c])
            .expect("failed to rebuild a captured region face");
    }

    let boundary_before = boundary_edges(&mesh_graph);
    assert!(
        !boundary_before.is_empty(),
        "captured region should have an outer boundary (the cut), got none"
    );

    #[cfg(feature = "rerun")]
    mesh_graph.log_rerun();

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();
    let _result = mesh_graph.merge_vertices_one_rings(
        vertex_ids[MERGE_V1],
        vertex_ids[MERGE_V2],
        FLIP_THRESHOLD_SQR,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        mesh_graph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    let boundary_after = boundary_edges(&mesh_graph);

    let capture_id = |v_id: VertexId| -> String {
        vertex_ids
            .iter()
            .position(|v| *v == v_id)
            .map(|i| i.to_string())
            .unwrap_or_else(|| format!("{v_id:?}"))
    };
    let added: Vec<String> = boundary_after
        .difference(&boundary_before)
        .map(|(a, b)| format!("({}, {})", capture_id(*a), capture_id(*b)))
        .collect();
    let removed: Vec<String> = boundary_before
        .difference(&boundary_after)
        .map(|(a, b)| format!("({}, {})", capture_id(*a), capture_id(*b)))
        .collect();
    assert!(
        boundary_after == boundary_before,
        "merge_vertices_one_rings changed the region's boundary: \
         before {} boundary edges, after {} (added {added:?}, removed {removed:?}). \
         The merge must not leave holes.",
        boundary_before.len(),
        boundary_after.len()
    );
}

#[test]
fn test_merge_one_ring_punch_cleanup_hole_reproduction() {
    get_tracing_subscriber();

    // Captured from a `log_002` (hole punch) run, journal step 1158: a
    // `merge_vertices_one_rings` inside `punch_hole` → `cleanup_mesh` →
    // `update_collisions_and_merge` ends its op on a still-closed region with
    // 3 boundary halfedges (HOLE DELTA probe: added 3, removed 0). The state
    // ring before the op (pos 1147-1157) was fully closed, so the merge itself
    // opened the surface — not the punch, which cuts after the cleanup loop.
    //
    // vertex id -> rebuild id mapping is the sorted array index (see comments).
    const REGION_VERTICES: [(f32, f32, f32); 26] = [
        (2.0573087, -5.599904, -50.18218),   // 2144v57 -> 0
        (2.3277264, -5.463469, -50.13234),   // 5610v23 -> 1
        (2.3339784, -5.4978213, -49.903564), // 6855v17 -> 2
        (2.3790693, -5.5867395, -50.40255),  // 7409v19 -> 3
        (2.015656, -5.5992184, -49.932896),  // 8355v17 -> 4
        (2.2156653, -5.389362, -50.060204),  // 9212v13 -> 5
        (1.9695282, -5.600073, -50.39962),   // 10434v21 -> 6
        (2.4975154, -5.786798, -50.113945),  // 10519v39 -> 7
        (2.0623565, -5.4420753, -49.941864), // 10736v19 -> 8
        (2.0732403, -5.276195, -50.452484),  // 10776v17 -> 9
        (2.1448529, -5.188434, -50.034267),  // 11208v19 -> 10
        (2.0802007, -5.446808, -50.169212),  // 11270v49 -> 11
        (1.9119468, -5.7574396, -50.362633), // 11358v47 -> 12
        (2.0073352, -5.4288244, -50.403534), // 12805v5 -> 13
        (2.317708, -5.3808737, -50.475094),  // 14922v13 -> 14
        (1.9286227, -5.7803106, -50.01557),  // 15749v25 -> 15
        (2.3845122, -5.7749863, -49.93603),  // 15864v5 -> 16
        (1.8896878, -5.813847, -50.222214),  // 16485v11 -> 17
        (2.0898988, -5.596813, -49.73825),   // 16846v23 -> 18
        (2.3438475, -5.661866, -49.78525),   // 17356v31 -> 19
        (2.2119386, -5.5473175, -49.820908), // 17627v29 -> 20
        (2.305505, -5.505895, -50.288124),   // 17959v39 -> 21
        (2.3471944, -5.6408277, -50.122337), // 19342v13 -> 22
        (2.1155572, -5.3494654, -50.307224), // 20116v41 -> 23
        (2.4244368, -5.674475, -50.283394),  // 20484v15 -> 24
        (2.1030927, -5.2937117, -50.156242), // 20718v27 -> 25
    ];
    const REGION_FACES: [(usize, usize, usize); 36] = [
        (0, 23, 11),
        (5, 2, 8),
        (15, 0, 4),
        (1, 2, 5),
        (24, 21, 3),
        (25, 5, 10),
        (17, 0, 15),
        (2, 1, 22),
        (2, 22, 16),
        (11, 8, 0),
        (5, 8, 10),
        (13, 0, 6),
        (20, 19, 18),
        (22, 24, 7),
        (19, 2, 16),
        (18, 8, 20),
        (23, 0, 13),
        (1, 21, 22),
        (23, 9, 14),
        (6, 0, 12),
        (20, 8, 2),
        (23, 21, 1),
        (23, 5, 25),
        (16, 22, 7),
        (2, 19, 20),
        (11, 23, 25),
        (4, 0, 8),
        (21, 24, 22),
        (14, 21, 23),
        (0, 17, 12),
        (3, 21, 14),
        (5, 23, 1),
        (8, 18, 4),
        (25, 10, 8),
        (23, 13, 9),
        (25, 8, 11),
    ];
    // The two vertices whose one-rings are merged (11270v49 -> 11, 5610v23 -> 1).
    const MERGE_V1: usize = 11;
    const MERGE_V2: usize = 1;
    // flip threshold of the punch cleanup (`log_002` sculpt params).
    const FLIP_THRESHOLD_SQR: f32 = 0.022667972;

    let mut mesh_graph = MeshGraph::new();

    let mut vertex_ids = Vec::with_capacity(REGION_VERTICES.len());
    for &(x, y, z) in &REGION_VERTICES {
        vertex_ids.push(mesh_graph.add_vertex(vec3(x, y, z)));
    }
    for &(a, b, c) in &REGION_FACES {
        mesh_graph
            .add_face_from_vertices(vertex_ids[a], vertex_ids[b], vertex_ids[c])
            .expect("failed to rebuild a captured region face");
    }

    let boundary_before = boundary_edges(&mesh_graph);
    assert!(
        !boundary_before.is_empty(),
        "captured region should have an outer boundary (the cut), got none"
    );

    #[cfg(feature = "rerun")]
    mesh_graph.log_rerun();

    let mut marked_halfedges = HashSet::new();
    let mut marked_vertices = HashSet::new();
    let _result = mesh_graph.merge_vertices_one_rings(
        vertex_ids[MERGE_V1],
        vertex_ids[MERGE_V2],
        FLIP_THRESHOLD_SQR,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        mesh_graph.log_rerun();
        RR.flush_blocking().unwrap();
    }

    let boundary_after = boundary_edges(&mesh_graph);

    let capture_id = |v_id: VertexId| -> String {
        vertex_ids
            .iter()
            .position(|v| *v == v_id)
            .map(|i| i.to_string())
            .unwrap_or_else(|| format!("{v_id:?}"))
    };
    let added: Vec<String> = boundary_after
        .difference(&boundary_before)
        .map(|(a, b)| format!("({}, {})", capture_id(*a), capture_id(*b)))
        .collect();
    let removed: Vec<String> = boundary_before
        .difference(&boundary_after)
        .map(|(a, b)| format!("({}, {})", capture_id(*a), capture_id(*b)))
        .collect();
    assert!(
        boundary_after == boundary_before,
        "merge_vertices_one_rings changed the region's boundary: \
         before {} boundary edges, after {} (added {added:?}, removed {removed:?}). \
         The merge must not leave holes.",
        boundary_before.len(),
        boundary_after.len()
    );
}

// Boundary edges of the region (the cut against the rest of the mesh), as
// undirected vertex pairs. The merge must not add or remove any of them.
fn boundary_edges(mesh: &MeshGraph) -> std::collections::BTreeSet<(VertexId, VertexId)> {
    let mut edges = std::collections::BTreeSet::new();
    for (_, he) in mesh.halfedges.iter().filter(|(_, h)| h.face.is_none()) {
        let Some(start) = he.start_vertex(mesh) else {
            continue;
        };
        let end = he.end_vertex;
        let edge = if start < end {
            (start, end)
        } else {
            (end, start)
        };
        edges.insert(edge);
    }
    edges
}
