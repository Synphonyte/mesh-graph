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

    assert_eq!(result.removed_faces.len(), 9);
    assert_eq!(result.removed_halfedges.len(), 22);
    assert_eq!(result.removed_vertices.len(), 3);

    assert_eq!(result.added_faces.len(), 3);
    assert_eq!(result.added_halfedges.len(), 4);
    assert_eq!(result.added_vertices.len(), 0);

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
