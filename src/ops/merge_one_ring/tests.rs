use super::MergeRecord;
use crate::hash::HashSet;
use crate::{
    utils::{extend_with, get_tracing_subscriber},
    *,
};
use glam::*;

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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_c_id, v_c_m_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_c_id, v_c_m_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(
        result.added_faces.len(),
        0,
        "the flip shortcut must not rebuild faces"
    );

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    // Note: `find_already_connected_strip` only takes the first strip of
    // existing faces, so this fixture seams one edge differently than a
    // two-strip teardown would. The invariants above still hold for it.
    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_vertices.len(), result.added_vertices.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_eq!(marked_vertices.len(), result.added_vertices.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let boundary_before = boundary_edges(&meshgraph);

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();

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

    assert_eq!(marked_halfedges.len(), result.added_halfedges.len());
    assert_merge_invariants(
        &meshgraph,
        &result,
        [v_top_id, v_bottom_id],
        &boundary_before,
        &marked_halfedges,
        &marked_vertices,
    );
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

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();
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

    drain_rim_taxonomy();
    assert_manifold_region(&mesh_graph);
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

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();
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

    drain_rim_taxonomy();
    assert_manifold_region(&mesh_graph);
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

/// Scoped invariants a successful one-ring merge must satisfy.
///
/// This replaces the per-fixture exact-count snapshots: the counts legitimately
/// change when the rebuild is reworked, but these properties must not. The
/// scoping is part of the contract:
///
/// - *New* boundary edges (undirected vertex pairs) are always a defect - they
///   are the hole this op opened. Boundary edges that existed before may
///   disappear: a merge on an open rim legitimately consumes them.
/// - Both merged vertices must be gone when the band rebuilt faces. The single
///   shared halfedge shortcut (flip + collapse) removes only one, so that path
///   is exempt.
/// - `wire` (an edge whose two halves are both face-less) and parallel edges
///   (two halfedges with the same start and end) are never valid.
/// - Added ids must be live, removed ids must be dead, the two sets must be
///   disjoint, and every marked id that survives the op must be live.
fn assert_merge_invariants(
    mesh: &MeshGraph,
    result: &MergeVerticesOneRing,
    merged: [VertexId; 2],
    boundary_before: &std::collections::BTreeSet<(VertexId, VertexId)>,
    marked_halfedges: &HashSet<HalfedgeId>,
    marked_vertices: &HashSet<VertexId>,
) {
    for &v_id in &result.added_vertices {
        assert!(
            mesh.vertices.contains_key(v_id),
            "added vertex {v_id:?} is not live"
        );
    }
    for &he_id in &result.added_halfedges {
        assert!(
            mesh.halfedges.contains_key(he_id),
            "added halfedge {he_id:?} is not live"
        );
    }
    for &face_id in &result.added_faces {
        assert!(
            mesh.faces.contains_key(face_id),
            "added face {face_id:?} is not live"
        );
    }

    for &v_id in &result.removed_vertices {
        assert!(
            !mesh.vertices.contains_key(v_id),
            "removed vertex {v_id:?} is still live"
        );
        assert!(
            !result.added_vertices.contains(&v_id),
            "vertex {v_id:?} is reported as both added and removed"
        );
    }
    for &he_id in &result.removed_halfedges {
        assert!(
            !mesh.halfedges.contains_key(he_id),
            "removed halfedge {he_id:?} is still live"
        );
        assert!(
            !result.added_halfedges.contains(&he_id),
            "halfedge {he_id:?} is reported as both added and removed"
        );
    }
    for &face_id in &result.removed_faces {
        assert!(
            !mesh.faces.contains_key(face_id),
            "removed face {face_id:?} is still live"
        );
        assert!(
            !result.added_faces.contains(&face_id),
            "face {face_id:?} is reported as both added and removed"
        );
    }

    let gone1 = !mesh.vertices.contains_key(merged[0]);
    let gone2 = !mesh.vertices.contains_key(merged[1]);
    if result.added_faces.is_empty() {
        assert!(
            gone1 || gone2,
            "merge removed neither of its input vertices {merged:?}"
        );
    } else {
        assert!(
            gone1 && gone2,
            "band merge left its input vertices behind: {merged:?}"
        );
    }

    let boundary_after = boundary_edges(mesh);
    let opened: Vec<(VertexId, VertexId)> = boundary_after
        .difference(boundary_before)
        .copied()
        .collect();
    assert!(
        opened.is_empty(),
        "merge opened new boundary edges: {opened:?}"
    );

    for (he_id, he) in mesh.halfedges.iter() {
        if he.face.is_none()
            && let Some(twin) = he.twin.and_then(|twin_id| mesh.halfedges.get(twin_id))
        {
            assert!(
                twin.face.is_some(),
                "wire edge (both halves face-less) at {he_id:?}"
            );
        }
    }

    for (v_id, out_hes) in &mesh.outgoing_halfedges {
        let mut ends: Vec<VertexId> = Vec::with_capacity(out_hes.len());
        for &he_id in out_hes {
            if let Some(he) = mesh.halfedges.get(he_id) {
                assert!(
                    !ends.contains(&he.end_vertex),
                    "parallel edges from {v_id:?} to {:?}",
                    he.end_vertex
                );
                ends.push(he.end_vertex);
            }
        }
    }

    for &he_id in marked_halfedges {
        assert!(
            mesh.halfedges.contains_key(he_id),
            "marked halfedge {he_id:?} is dead"
        );
    }
    for &v_id in marked_vertices {
        assert!(
            mesh.vertices.contains_key(v_id),
            "marked vertex {v_id:?} is dead"
        );
    }

    // Phase 0/1 rim taxonomy: drain this test's probe records (thread-local).
    // The end-of-op sample is the wire gate: whatever the planner samples
    // pre-net, the op must not leave wire edges behind. Printed only on demand.
    let probes = super::rim_probe::drain();
    for probe in &probes {
        assert!(
            probe.end_wire.is_empty(),
            "merge op ended with wire edges: {}",
            probe.summary()
        );
    }
    if std::env::var("MESH_GRAPH_RIM_TAXONOMY").is_ok() {
        for probe in &probes {
            println!("RIM {}", probe.summary());
        }
    }
}

/// The final mesh must not contain parallel edges (two halfedges with the same
/// start and end) or wire edges (both halves face-less). The boundary-set
/// comparison alone cannot see them: a parallel leftover whose undirected pair
/// already exists adds no new boundary edge, and a vertex split into coincident
/// duplicates keeps the pair set identical.
fn assert_manifold_region(mesh: &MeshGraph) {
    for (v_id, out_hes) in mesh.outgoing_halfedges.iter() {
        let mut ends: Vec<VertexId> = Vec::with_capacity(out_hes.len());
        for &he_id in out_hes {
            if let Some(he) = mesh.halfedges.get(he_id) {
                assert!(
                    !ends.contains(&he.end_vertex),
                    "parallel edges from {v_id:?} to {:?}",
                    he.end_vertex
                );
                ends.push(he.end_vertex);
            }
        }
    }

    for (he_id, he) in mesh.halfedges.iter() {
        if he.face.is_none()
            && let Some(twin) = he.twin.and_then(|twin_id| mesh.halfedges.get(twin_id))
        {
            assert!(
                twin.face.is_some(),
                "wire edge (both halves face-less) at {he_id:?}"
            );
        }
    }
}

/// Drain and, with `MESH_GRAPH_RIM_TAXONOMY=1`, print this thread's rim probes.
fn drain_rim_taxonomy() {
    let label = std::thread::current()
        .name()
        .unwrap_or("<unnamed>")
        .to_string();
    let probes = super::rim_probe::drain();
    if std::env::var("MESH_GRAPH_RIM_TAXONOMY").is_ok() {
        for probe in &probes {
            println!("RIM {label} {}", probe.summary());
        }
    }
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

/// Regression test for the weld-hole defect: the morphological close
/// (freestyle-sculpt `test_apply_weld_amount_applies_close`) intermittently
/// leaves 4 boundary halfedges after fusing two tessellated cubes. The HOLE
/// probe (`MESH_GRAPH_HOLE_CHECK=1`) attributes it to a single
/// `merge_vertices_one_rings` whose stitch band closes with a hole.
///
/// Currently fails (reproduces the defect) until the merge is fixed.
///
/// The region below was extracted with `cargo run --bin extract-merge-region`
/// from the probe dump captured on that fixture (journal step 500, merge of
/// vertices 367 and 1083 of that run): the faces within graph distance 1 of the
/// merge pair from the last verified state — 17 vertices / 23 faces, all
/// triangles, no parallel halfedge pairs. Rebuilding exactly that region and
/// replaying the single merge reproduces the defect deterministically: the
/// merge removes the two vertices and stitches a band (6 added faces), but
/// leaves 4 additional boundary halfedges (hole) — on the extracted region the
/// outer cut grows from 5 to 9 boundary edges.
///
/// `FLIP_THRESHOLD_SQR` is the weld's `SculptParams::from_mesh_graph(fixture,
/// 0.2).min_edge_length_squared` (median edge 0.125 -> max edge 0.2).
#[test]
fn test_merge_one_ring_close_hole_reproduction() {
    get_tracing_subscriber();

    // vertex id -> rebuild id mapping is the sorted array index (see comments).
    const REGION_VERTICES: [(f32, f32, f32); 17] = [
        (1.4370954, 0.451704, 1.0509045),  // 1223 -> 0
        (1.4375, 0.375, 0.625),            // 368 -> 1
        (1.4487758, 0.2842505, 1.0250168), // 1222 -> 2
        (1.4677734, 0.374468, 0.4689652),  // 692 -> 3
        (1.4938163, 0.4002733, 0.9384441), // 382 -> 4
        (1.4949148, 0.2527521, 0.4949971), // 685 -> 5
        (1.499954, 0.2532232, 0.6329052),  // 367 -> 6 (merge vertex 1)
        (1.5, 0.484375, 0.6172814),        // 700 -> 7
        (1.5031247, 0.2185288, 0.8041879), // 1083 -> 8 (merge vertex 2)
        (1.5127819, 0.4008933, 0.7652988), // 375 -> 9
        (1.5127819, 0.4008933, 0.7652988), // 694 -> 10
        (1.540801, 0.2279671, 0.9078187),  // 688 -> 11
        (1.5425725, 0.5086495, 1.0033165), // 1240 -> 12
        (1.5455617, 0.2298947, 1.0587322), // 1237 -> 13
        (1.5506089, 0.4160118, 0.8858563), // 695 -> 14
        (1.5625, 0.375, 0.625),            // 693 -> 15
        (1.5834919, 0.3437378, 1.040552),  // 1238 -> 16
    ];
    const REGION_FACES: [(usize, usize, usize); 23] = [
        (4, 9, 12),
        (5, 3, 1),
        (5, 1, 6),
        (3, 7, 1),
        (4, 12, 0),
        (6, 1, 10),
        (1, 7, 10),
        (8, 9, 4),
        (8, 4, 11),
        (4, 0, 2),
        (14, 12, 9),
        (5, 6, 15),
        (5, 15, 3),
        (6, 10, 15),
        (8, 11, 14),
        (8, 14, 9),
        (11, 16, 14),
        (3, 15, 7),
        (15, 10, 7),
        (2, 11, 4),
        (11, 13, 16),
        (14, 16, 12),
        (13, 11, 2),
    ];
    // The two vertices whose one-rings are merged (367 -> 6, 1083 -> 8).
    const MERGE_V1: usize = 6;
    const MERGE_V2: usize = 8;
    // min_edge_length_squared of the weld's SculptParams (see doc comment).
    const FLIP_THRESHOLD_SQR: f32 = 0.0096;

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

    let mut marked_halfedges = HashSet::default();
    let mut marked_vertices = HashSet::default();
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

    drain_rim_taxonomy();
    assert_manifold_region(&mesh_graph);
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

// ---------------------------------------------------------------------------
// The lens seam region.
//
// Captured from the weld (freestyle-sculpt `test_apply_weld_amount_applies_close`)
// fusing two tessellated cubes wall-to-wall, at its merge of vertices 1218 and
// 1243: the faces around the merge pair, 30 vertices / 41 faces, cut out of the
// full mesh (so the region has an outer boundary - the cut - that no correct
// merge may touch).
//
// The two one-rings share *two* vertices, 720 and 1217, which makes this a
// *lens*: the band between the rings falls into two segments, one on each side
// of the shared pair.
// ---------------------------------------------------------------------------

const LENS_REGION_VERTICES: [(f32, f32, f32); 30] = [
    (1.30936, 1.30936, 0.3749998),   // 284 -> 0
    (1.30936, 1.30936, 0.5),         // 285 -> 1
    (0.0, 0.0, 0.0),                 // 347 -> 2
    (0.0, 0.0, 0.0),                 // 358 -> 3
    (1.690641, 1.30936, 0.3749986),  // 616 -> 4
    (1.69064, 1.309359, 0.5),        // 618 -> 5
    (1.509474, 1.178084, 0.1444791), // 718 -> 6
    (0.0, 0.0, 0.0),                 // 720 -> 7
    (0.0, 0.0, 0.0),                 // 721 -> 8
    (0.0, 0.0, 0.0),                 // 1217 -> 9
    (1.416506, 1.040559, 0.4062499), // 1218 -> 10
    (1.416506, 1.040559, 0.53125),   // 1219 -> 11
    (1.483415, 1.033147, 0.6533897), // 1220 -> 12
    (1.583494, 1.040559, 0.4062499), // 1243 -> 13
    (1.583494, 1.040559, 0.53125),   // 1244 -> 14
    (1.514609, 1.167312, 0.8002385), // 1245 -> 15
    (1.378634, 1.154926, 0.6093761), // 1689 -> 16
    (1.621366, 1.154926, 0.6093746), // 1714 -> 17
    (1.378633, 1.154926, 0.3593675), // 1738 -> 18
    (1.378634, 1.154926, 0.484375),  // 1744 -> 19
    (1.621366, 1.154926, 0.484375),  // 1779 -> 20
    (1.621362, 1.154915, 0.3593882), // 1806 -> 21
    (1.422675, 1.176207, 0.2161386), // 1828 -> 22
    (1.584667, 1.165663, 0.2412385), // 1848 -> 23
    (1.655983, 1.232118, 0.304426),  // 1935 -> 24
    (1.343996, 1.232143, 0.3044841), // 1945 -> 25
    (1.656004, 1.232143, 0.4296868), // 1975 -> 26
    (1.343997, 1.232143, 0.554688),  // 1980 -> 27
    (1.343997, 1.232143, 0.4296874), // 1996 -> 28
    (1.656003, 1.232142, 0.5546873), // 2016 -> 29
];
const LENS_REGION_FACES: [(usize, usize, usize); 41] = [
    (9, 23, 22),
    (25, 0, 18),
    (9, 3, 6),
    (2, 3, 7),
    (28, 1, 19),
    (10, 7, 3),
    (11, 8, 7),
    (6, 3, 2),
    (13, 9, 2),
    (14, 13, 7),
    (15, 14, 8),
    (18, 10, 9),
    (19, 11, 10),
    (16, 12, 11),
    (6, 2, 9),
    (24, 23, 21),
    (2, 7, 13),
    (26, 21, 20),
    (7, 8, 14),
    (29, 20, 17),
    (3, 9, 10),
    (7, 10, 11),
    (8, 11, 12),
    (22, 18, 9),
    (9, 13, 21),
    (18, 19, 10),
    (19, 16, 11),
    (13, 14, 20),
    (14, 15, 17),
    (28, 18, 0),
    (27, 19, 1),
    (14, 17, 20),
    (13, 20, 21),
    (9, 21, 23),
    (21, 4, 24),
    (18, 22, 25),
    (4, 21, 26),
    (20, 5, 26),
    (16, 19, 27),
    (19, 18, 28),
    (5, 20, 29),
];

/// The merged pair (1218 -> 10, 1243 -> 13).
const LENS_MERGE_V1: usize = 10;
const LENS_MERGE_V2: usize = 13;
/// The weld's `SculptParams::min_edge_length_squared` (0.0096).
const LENS_FLIP_THRESHOLD_SQR: f32 = 0.0096;

/// The five innermost seam vertices (1217, 358, 347, 720, 721 -> indices 9, 3,
/// 2, 7, 8) have no entry in the captured `positions` map, so the capture above
/// carries them as `(0, 0, 0)`. That is an artifact of the capture, not of the
/// mesh: five coincident vertices at the origin make the merge's
/// position-coincident weld fuse the whole seam and turn every face through it
/// degenerate, which is nothing the real mesh ever did.
///
/// These are those five positions reconstructed by Laplacian smoothing against
/// their (known) neighbors - plausible, smooth, and above all non-degenerate.
/// They are a reconstruction, so they pin geometry, not a captured fact; what
/// they buy is a region on which "the merge must not open a hole" is a fair
/// question to ask.
const LENS_RECONSTRUCTED_SEAM: [(usize, (f32, f32, f32)); 5] = [
    (2, (1.517155, 1.101042, 0.326519)),
    (3, (1.489323, 1.101042, 0.326519)),
    (7, (1.500895, 1.061858, 0.445774)),
    (8, (1.499784, 1.068687, 0.592380)),
    (9, (1.502588, 1.123666, 0.309572)),
];

/// Rebuilds the region. With `reconstruct_seam`, the five seam vertices the
/// capture lost get `LENS_RECONSTRUCTED_SEAM` instead of the origin.
fn build_lens_region(reconstruct_seam: bool) -> (MeshGraph, Vec<VertexId>) {
    build_lens_region_into(MeshGraph::new(), reconstruct_seam)
}

/// [`build_lens_region`] into a given empty mesh (e.g. one from
/// `utils::mesh_with_key_generation`).
fn build_lens_region_into(
    mut mesh_graph: MeshGraph,
    reconstruct_seam: bool,
) -> (MeshGraph, Vec<VertexId>) {
    let mut positions = LENS_REGION_VERTICES;
    if reconstruct_seam {
        for (idx, pos) in LENS_RECONSTRUCTED_SEAM {
            positions[idx] = pos;
        }
    }

    let mut vertex_ids = Vec::with_capacity(positions.len());
    for &(x, y, z) in &positions {
        vertex_ids.push(mesh_graph.add_vertex(vec3(x, y, z)));
    }
    for &(a, b, c) in &LENS_REGION_FACES {
        mesh_graph
            .add_face_from_vertices(vertex_ids[a], vertex_ids[b], vertex_ids[c])
            .expect("failed to rebuild a captured region face");
    }

    (mesh_graph, vertex_ids)
}

/// Replays the captured merge on a freshly rebuilt region.
fn merge_lens_region(reconstruct_seam: bool) -> (MeshGraph, Vec<VertexId>) {
    merge_lens_region_into(MeshGraph::new(), reconstruct_seam)
}

/// [`merge_lens_region`] on a region built into a given empty mesh.
fn merge_lens_region_into(mesh: MeshGraph, reconstruct_seam: bool) -> (MeshGraph, Vec<VertexId>) {
    let (mut mesh_graph, vertex_ids) = build_lens_region_into(mesh, reconstruct_seam);

    let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
    let mut marked_vertices: HashSet<VertexId> = HashSet::default();
    // The captured run had vertex 1217 (index 9) in the topology manager's
    // protected set; keep that part of the input.
    marked_vertices.insert(vertex_ids[9]);

    mesh_graph.merge_vertices_one_rings(
        vertex_ids[LENS_MERGE_V1],
        vertex_ids[LENS_MERGE_V2],
        LENS_FLIP_THRESHOLD_SQR,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    (mesh_graph, vertex_ids)
}

/// A lens merge must not open a hole.
///
/// The short segment of the lens is one step on each ring (here 358 -> 1217
/// against 347 -> 1217). The faces that covered it are torn out with the merged
/// vertices' stars, so a band that drops the segment leaves a triangular hole at
/// 347/358/1217.
#[test]
fn test_merge_one_ring_lens_stays_watertight() {
    get_tracing_subscriber();

    let (before, _) = build_lens_region(true);
    let boundary_before = boundary_edges(&before);
    assert!(
        !boundary_before.is_empty(),
        "captured region should have an outer boundary (the cut), got none"
    );

    let (mesh_graph, vertex_ids) = merge_lens_region(true);
    drain_rim_taxonomy();
    assert_manifold_region(&mesh_graph);
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
        "the lens merge changed the region's boundary: before {} edges, after {} \
         (added {added:?}, removed {removed:?}). The merge must not leave holes.",
        boundary_before.len(),
        boundary_after.len(),
    );
}

/// The lens merge must not depend on hash iteration order.
///
/// Several steps of the merge and of the vertex-neighborhood cleanup used to
/// pick "an element" out of a hash table (the fan-walk start, the flap candidate
/// pair, the order faces and halfedges are freed in). Those picks followed the
/// table's iteration order rather than the mesh, and so did the repair. Every
/// such choice is driven by mesh state now, so the merge must give identical
/// topology down to the reused slot indices. Before the fix the degenerate
/// variant produced 14 different outcomes in 200 runs (with randomly seeded
/// tables).
///
/// The hash seed is fixed now, so the region is rebuilt under keys of different
/// versions instead (`mesh_with_key_generation`): same topology, different
/// hashes, so any dependence on iteration order changes the result.
#[test]
fn test_merge_one_ring_lens_is_order_independent() {
    const GENERATIONS: u32 = 32;

    get_tracing_subscriber();

    for reconstruct_seam in [false, true] {
        let mut fingerprints = std::collections::BTreeSet::<String>::new();

        for generation in 0..GENERATIONS {
            let (mesh_graph, _) = merge_lens_region_into(
                crate::utils::mesh_with_key_generation(generation),
                reconstruct_seam,
            );
            fingerprints.insert(crate::utils::topology_fingerprint(&mesh_graph));
        }

        assert_eq!(
            fingerprints.len(),
            1,
            "the lens merge is not order-independent (reconstructed seam: \
             {reconstruct_seam}): the same input under {GENERATIONS} key generations \
             produced {} different results:\n{}",
            fingerprints.len(),
            fingerprints
                .iter()
                .map(|f| format!("  {f}"))
                .collect::<Vec<_>>()
                .join("\n"),
        );
    }
}

/// A planned face must not leave a *parallel* edge behind.
///
/// Captured from the weld at the merge that first opened the mesh: 29 vertices /
/// 39 faces around the merge pair, cut out of the live fixture, with the
/// protected vertex of that run kept.
///
/// One of the band's planned faces needs the directed slot `a -> b` on an edge
/// whose only free half points the other way (an earlier face of the same merge
/// took it). `add_or_get_boundary_edge` answers that by adding a *second* edge
/// between the same two vertices, and here nothing ever claimed its two free
/// halves: they stayed face-less, so the mesh reported them as boundary - a hole
/// with no face missing from it - while the two triangles that should share the
/// edge each sat on their own copy. The merge now fuses such leftovers
/// (`weld_parallel_edges`).
#[test]
fn test_merge_one_ring_parallel_edge_leftover_stays_watertight() {
    get_tracing_subscriber();

    const REGION_VERTICES: [(f32, f32, f32); 29] = [
        (1.1929264, 0.25957376, 1.0725257),
        (1.6030791, 0.016003627, 0.7632272),
        (1.2991978, 0.14543079, 0.9672159),
        (1.3917341, 0.3625335, 0.9987495),
        (1.5548906, 0.014888145, 0.6118068),
        (1.2856488, -0.04666477, 0.66112864),
        (1.6209815, 0.012684029, 0.8143895),
        (1.4307325, 0.006833475, 0.8788141),
        (1.3300436, -0.017816246, 0.81030643),
        (1.4403498, 0.047905367, 0.93121576),
        (1.4681315, 0.19360392, 0.9283303),
        (1.2133489, 0.18374455, 1.0048174),
        (1.3918209, -0.00883111, 0.6978003),
        (1.6017299, 0.2138994, 0.9127206),
        (1.2960103, 0.3831477, 1.0212176),
        (1.3850467, 0.10711703, 0.9296145),
        (1.3946521, -0.021617379, 0.5885098),
        (1.1814737, -0.012920812, 0.9872659),
        (1.2171628, 0.039629802, 1.012486),
        (1.4254899, 0.018803619, 0.8114579),
        (1.3579687, 0.24615975, 0.9701845),
        (1.5209364, 0.055423483, 0.8126094),
        (1.5379136, 0.035155814, 0.7122081),
        (1.487458, 0.3419193, 0.9762814),
        (1.482294, 0.07301916, 0.9232739),
        (1.1994991, -0.07863506, 0.7529068),
        (1.3011584, 0.056889433, 0.94549817),
        (1.5795412, 0.0389213, 0.9169333),
        (1.2427127, -0.042345867, 0.89821833),
    ];
    const REGION_FACES: [(usize, usize, usize); 39] = [
        (14, 20, 3),
        (0, 20, 14),
        (18, 17, 26),
        (5, 8, 25),
        (19, 12, 21),
        (21, 22, 1),
        (7, 26, 28),
        (27, 10, 24),
        (9, 15, 26),
        (2, 20, 11),
        (3, 20, 23),
        (13, 23, 10),
        (6, 27, 21),
        (11, 26, 2),
        (21, 7, 19),
        (15, 20, 2),
        (12, 5, 16),
        (28, 26, 17),
        (8, 12, 19),
        (25, 8, 28),
        (20, 0, 11),
        (20, 15, 10),
        (26, 7, 9),
        (21, 12, 22),
        (10, 23, 20),
        (24, 9, 27),
        (12, 16, 4),
        (15, 9, 24),
        (2, 26, 15),
        (1, 6, 21),
        (9, 7, 27),
        (8, 5, 12),
        (26, 11, 18),
        (7, 28, 8),
        (22, 12, 4),
        (7, 21, 27),
        (10, 27, 13),
        (19, 7, 8),
        (24, 10, 15),
    ];

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
        "region should have an outer cut"
    );

    let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
    let mut marked_vertices: HashSet<VertexId> = HashSet::default();
    marked_vertices.insert(vertex_ids[7]);

    mesh_graph.merge_vertices_one_rings(
        vertex_ids[19],
        vertex_ids[15],
        0.0096,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    drain_rim_taxonomy();
    assert_manifold_region(&mesh_graph);
    let boundary_after = boundary_edges(&mesh_graph);
    assert!(
        boundary_after == boundary_before,
        "the merge changed the region's boundary: before {} edges, after {}. \
         The merge must not leave holes.",
        boundary_before.len(),
        boundary_after.len(),
    );
}

/// The band must reach the shared vertex on *both* rings.
///
/// Another lens - the one-rings share vertices 19 and 17 - but an asymmetric
/// one: the two shared vertices are adjacent on ring 1 and four steps apart on
/// ring 2. A band that pairs the rings in lockstep covers one step of ring 2's
/// arc instead of all four and leaves an open quad at 17/22/30/26.
#[test]
fn test_merge_one_ring_asymmetric_lens_stays_watertight() {
    get_tracing_subscriber();

    const REGION_VERTICES: [(f32, f32, f32); 33] = [
        (1.3093596, 0.5, 1.3093596),
        (1.3093692, 0.6249998, 1.3093499),
        (1.4374995, 0.2496348, 0.7503662),
        (1.4374998, 0.37499997, 0.75037587),
        (1.4374998, 0.5, 0.750376),
        (1.4367635, 0.37499925, 0.898358),
        (1.4367634, 0.5, 0.89835876),
        (1.5008514, 0.59192085, 0.8613802),
        (1.5625005, 0.24963538, 0.7503645),
        (1.5077914, 0.27509907, 0.88640124),
        (1.358845, 0.3296609, 1.2320743),
        (1.5625002, 0.375, 0.7503758),
        (1.5632364, 0.37499958, 0.89835626),
        (1.5625002, 0.50000006, 0.750376),
        (1.5301834, 0.49542326, 0.90014625),
        (1.4945834, 0.5461893, 0.95348686),
        (1.3551339, 0.7116632, 1.2293802),
        (1.4883152, 0.5004578, 1.0455935),
        (1.5445898, 0.41506463, 1.1139808),
        (1.5055845, 0.33813843, 1.0295433),
        (1.4165065, 0.40624997, 1.040559),
        (1.4670306, 0.60281163, 1.1013482),
        (1.3743107, 0.52626896, 1.1771944),
        (1.4142686, 0.1871017, 1.1966355),
        (1.6008643, 0.32967144, 1.182368),
        (1.6019799, 0.5948021, 1.174322),
        (1.4380281, 0.33481583, 1.1546514),
        (1.6213715, 0.45300648, 1.1549394),
        (1.5692087, 0.20108268, 1.1939099),
        (1.4457461, 0.70516545, 1.1571027),
        (1.4061694, 0.4305424, 1.1659229),
        (1.3439966, 0.4296874, 1.2321427),
        (1.5257758, 0.4377287, 0.97197485),
    ];
    const REGION_FACES: [(usize, usize, usize); 45] = [
        (3, 5, 2),
        (5, 9, 2),
        (4, 6, 3),
        (6, 5, 3),
        (7, 6, 4),
        (20, 19, 5),
        (17, 20, 6),
        (9, 12, 8),
        (12, 11, 8),
        (19, 12, 9),
        (12, 14, 11),
        (14, 13, 11),
        (32, 14, 12),
        (14, 7, 13),
        (21, 22, 17),
        (9, 5, 19),
        (10, 23, 26),
        (5, 6, 20),
        (30, 31, 26),
        (15, 6, 7),
        (12, 19, 32),
        (16, 22, 29),
        (22, 31, 30),
        (19, 26, 24),
        (32, 19, 17),
        (29, 22, 21),
        (15, 14, 17),
        (26, 31, 10),
        (18, 27, 17),
        (18, 19, 24),
        (27, 25, 17),
        (19, 20, 26),
        (24, 26, 28),
        (20, 17, 22),
        (22, 1, 0),
        (17, 25, 21),
        (17, 19, 18),
        (7, 14, 15),
        (23, 28, 26),
        (30, 20, 22),
        (17, 6, 15),
        (1, 22, 16),
        (26, 20, 30),
        (22, 0, 31),
        (17, 14, 32),
    ];

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
        "region should have an outer cut"
    );

    let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
    let mut marked_vertices: HashSet<VertexId> = HashSet::default();
    for idx in [17usize, 19, 21] {
        marked_vertices.insert(vertex_ids[idx]);
    }

    mesh_graph.merge_vertices_one_rings(
        vertex_ids[32],
        vertex_ids[20],
        0.0096,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    drain_rim_taxonomy();
    assert_manifold_region(&mesh_graph);
    let boundary_after = boundary_edges(&mesh_graph);
    assert!(
        boundary_after == boundary_before,
        "the merge changed the region's boundary: before {} edges, after {}. \
         The merge must not leave holes.",
        boundary_before.len(),
        boundary_after.len(),
    );
}

/// A merge case extracted from the weld, minimised to the geometry that carries
/// the defect.
struct WeldCase {
    /// What makes this one different from the others.
    what: &'static str,
    verts: &'static [(f32, f32, f32)],
    faces: &'static [(usize, usize, usize)],
    /// Per vertex whose whole star is inside the cut: the order of its outgoing
    /// halfedges as neighbor indices, and the neighbor its `outgoing_halfedge`
    /// seed points at.
    ///
    /// This is part of the input, not bookkeeping. `one_ring()` starts at the
    /// seed and the band planner scans the ring from index 0, so the order
    /// decides where the band is cut. A rebuild derives it from the order faces
    /// happen to be added, which is not the order the live mesh had - restore it
    /// or the case is a different problem from the one that failed. (Every cut
    /// that "failed to reproduce" during this hunt was missing exactly this.)
    outgoing: &'static [(usize, &'static [usize], usize)],
    /// Indices of the two vertices whose one-rings are merged.
    merge: (usize, usize),
    /// Indices the capture run had in the topology manager's protected set.
    protected: &'static [usize],
}

impl WeldCase {
    fn build(&self) -> (MeshGraph, Vec<VertexId>) {
        let mut mesh_graph = MeshGraph::new();

        let mut vertex_ids = Vec::with_capacity(self.verts.len());
        for &(x, y, z) in self.verts {
            vertex_ids.push(mesh_graph.add_vertex(vec3(x, y, z)));
        }
        for &(a, b, c) in self.faces {
            mesh_graph
                .add_face_from_vertices(vertex_ids[a], vertex_ids[b], vertex_ids[c])
                .expect("failed to rebuild a captured region face");
        }

        for (idx, ends, seed) in self.outgoing {
            let v_id = vertex_ids[*idx];
            let Some(out_hes) = mesh_graph.outgoing_halfedges.get(v_id).cloned() else {
                continue;
            };

            let mut reordered = Vec::with_capacity(out_hes.len());
            let mut left = out_hes;
            for end_idx in *ends {
                let want = vertex_ids[*end_idx];
                if let Some(pos) = left.iter().position(|he_id| {
                    mesh_graph
                        .halfedges
                        .get(*he_id)
                        .is_some_and(|he| he.end_vertex == want)
                }) {
                    reordered.push(left.remove(pos));
                }
            }
            reordered.extend(left);

            let seed_v = vertex_ids[*seed];
            let seed_he = reordered.iter().copied().find(|he_id| {
                mesh_graph
                    .halfedges
                    .get(*he_id)
                    .is_some_and(|he| he.end_vertex == seed_v)
            });

            mesh_graph.outgoing_halfedges.insert(v_id, reordered);
            if let Some(seed_he) = seed_he
                && let Some(vert) = mesh_graph.vertices.get_mut(v_id)
            {
                vert.outgoing_halfedge = Some(seed_he);
            }
        }

        (mesh_graph, vertex_ids)
    }

    /// Replays the merge and returns `(boundary before, boundary after, newly
    /// opened *interior* edges)` as capture indices.
    ///
    /// Only interior edges count as a defect. A captured region is a cut out of
    /// a larger mesh, so its rim is boundary here but not in the mesh it came
    /// from - and the ops treat boundary specially (`merge_remaining_unconnected`
    /// fuses vertices joined by boundary halfedges, which in the real mesh could
    /// never fire there). What the rim does is an artifact of the cut; a new
    /// boundary edge between two vertices that were both *interior* is not.
    fn replay(&self) -> (usize, usize, Vec<String>) {
        let (mut mesh_graph, vertex_ids) = self.build();

        let boundary_before = boundary_edges(&mesh_graph);

        let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
        let mut marked_vertices: HashSet<VertexId> = HashSet::default();
        for idx in self.protected {
            marked_vertices.insert(vertex_ids[*idx]);
        }

        mesh_graph.merge_vertices_one_rings(
            vertex_ids[self.merge.0],
            vertex_ids[self.merge.1],
            // The weld's `SculptParams::min_edge_length_squared`.
            0.0096,
            &mut marked_halfedges,
            &mut marked_vertices,
        );

        let boundary_after = boundary_edges(&mesh_graph);

        let capture_id = |v_id: VertexId| -> String {
            vertex_ids
                .iter()
                .position(|v| *v == v_id)
                .map(|i| i.to_string())
                .unwrap_or_else(|| format!("{v_id:?}"))
        };

        let rim: std::collections::BTreeSet<VertexId> =
            boundary_before.iter().flat_map(|(a, b)| [*a, *b]).collect();

        let mut opened: Vec<String> = boundary_after
            .difference(&boundary_before)
            .filter(|(a, b)| !rim.contains(a) && !rim.contains(b))
            .map(|(a, b)| format!("({}, {})", capture_id(*a), capture_id(*b)))
            .collect();
        opened.sort();

        (boundary_before.len(), boundary_after.len(), opened)
    }
}

/// Weld merges that used to open a watertight region, kept as regression cases.
///
/// Each was extracted from a failing `weld_flake_loop_until_failure` run with
/// `WELD_CAPTURE=1` (freestyle-sculpt): the capture cuts the neighborhood of the
/// merge that first opened the mesh, widens the cut until the merge reproduces
/// on it standalone *with both merged vertices still interior*, then drops faces
/// for as long as it keeps reproducing - never a face of the merge pair's or
/// their one-rings' stars, since those are the op's actual input.
///
/// They are deterministic even though the weld that produced them is not: a
/// region rebuilt in index order gets its ids from the rebuild, the outgoing
/// order is restored from the capture, and the ops are order-independent.
///
/// `radius 1, +5` and `radius 1, +6` moved here from [`OPEN_WELD_CASES`] when the
/// rim-first planner closed them; see the module docs of
/// [`super::rim_planner`] for how.
/// The gap=0.1 weld's residual hole: a merge opens six boundary halfedges, its own
/// closer now takes three of them, and the last three survive. Captured at radius 1.
const MERGE_GAP01_TRIANGLE: WeldCase = WeldCase {
    what: "gap=0.1 weld: the merge leaves a triangle hole",
    verts: &[
        (0.7750727, 0.61499834, -0.03464932),
        (0.8723206, 0.5960623, -0.027743543),
        (0.7777778, 0.44444442, -0.05),
        (0.9642105, 0.55365837, -0.032252),
        (0.95671344, 0.34631467, -0.045972914),
        (1.05, 0.22222222, 0.11111111),
        (1.05, 0.22222222, 0.22222222),
        (1.051393, 0.3640269, 0.21421906),
        (0.94400084, 0.68171823, -0.0048410837),
        (0.9169391, 0.47784942, -0.04338023),
        (1.05, 0.33333334, 0.33333334),
        (1.05, 0.44444445, 0.44444445),
        (1.051393, 0.3640269, 0.21421906),
        (0.8672456, 0.39537954, -0.047986455),
        (1.038849, 0.622871, -0.024694977),
        (1.1717129, 0.63949275, -0.046218757),
        (1.1402444, 0.42647782, -0.03837791),
        (1.0484788, 0.38639623, -0.042175412),
        (1.1703477, 0.53000826, -0.042907573),
        (1.0500001, 0.22222222, 0.11111111),
        (1.0500001, 0.22222222, 0.22222222),
        (1.065086, 0.34535992, 0.08348932),
        (1.0500001, 0.33333334, 0.33333334),
        (1.0427585, 0.48097044, 0.106952205),
        (1.0531566, 0.53851026, 0.3659184),
        (1.055572, 0.4561075, 0.30132067),
        (1.0500001, 0.44444445, 0.44444445),
        (1.0697759, 0.756099, -0.003692946),
        (1.0561004, 0.51125443, -0.036760457),
    ],
    faces: &[
        (7, 12, 23),
        (2, 9, 13),
        (3, 9, 1),
        (8, 1, 0),
        (13, 9, 4),
        (21, 7, 5),
        (7, 6, 5),
        (7, 23, 25),
        (1, 8, 14),
        (7, 10, 6),
        (25, 10, 7),
        (25, 11, 10),
        (24, 11, 25),
        (25, 26, 24),
        (28, 15, 18),
        (1, 14, 3),
        (14, 15, 28),
        (18, 16, 28),
        (28, 4, 9),
        (2, 1, 9),
        (17, 28, 16),
        (20, 12, 19),
        (12, 21, 19),
        (22, 12, 20),
        (22, 25, 12),
        (26, 25, 22),
        (28, 9, 3),
        (27, 15, 14),
        (25, 23, 12),
        (14, 8, 27),
        (1, 2, 0),
        (7, 21, 12),
        (4, 28, 17),
        (3, 14, 28),
    ],
    outgoing: &[
        (1, &[8, 2, 3, 14, 0, 9], 0),
        (3, &[1, 14, 28, 9], 1),
        (7, &[21, 5, 6, 25, 10, 23, 12], 12),
        (9, &[2, 1, 28, 4, 13, 3], 2),
        (12, &[20, 19, 22, 21, 25, 23, 7], 7),
        (14, &[28, 1, 27, 15, 8, 3], 28),
        (23, &[25, 12, 7], 25),
        (25, &[23, 24, 7, 10, 11, 22, 26, 12], 7),
        (28, &[3, 9, 4, 18, 14, 16, 15, 17], 3),
    ],
    merge: (23, 3),
    protected: &[17, 27, 28],
};

/// The one weld case that takes the merge's fast path (the flip-plus-collapse of the single
/// shared edge) instead of planning a band. It is a weld case because its region closes -
/// the op now closes the four halfedges its coincident fusion opened - but the band-facing
/// tests have no band to measure for it.
const FAST_PATH_CASE: &str = "gap=0.05 weld: two shared ring vertices, band leaves the region open";

const WELD_CASES: &[WeldCase] = &[
    WELD_GAP005_MERGE,
    WeldCase {
        what: "band wraps the full ring and leaves one seam triangle",
        verts: &[
            (1.498734, 0.50585926, 0.11197451),
            (1.4489955, 0.47922638, 0.26759854),
            (1.4374998, 0.625, 0.24962415),
            (1.4659085, 0.22209845, 0.35202032),
            (1.4375, 0.5, 0.37499997),
            (1.4375, 0.625, 0.37499997),
            (1.4464133, 0.46001363, 0.48747584),
            (1.4456786, 0.58433324, 0.4792942),
            (1.4374998, 0.750376, 0.5),
            (1.5095625, 0.7555237, 0.55578065),
            (1.4672023, 0.35099435, 0.49793303),
            (1.5, 0.25328678, 0.25328594),
            (1.5625002, 0.24962428, 0.37500024),
            (1.4879913, 0.24197507, 0.5083902),
            (1.5625002, 0.37500012, 0.24962416),
            (1.5625, 0.37499997, 0.37499997),
            (1.5625, 0.37499997, 0.5),
            (1.5625002, 0.5, 0.24962403),
            (1.5625, 0.5, 0.37499997),
            (1.5625, 0.5, 0.5),
            (1.5037155, 0.460831, 0.62551266),
            (1.5625002, 0.625, 0.24962415),
            (1.5625, 0.625, 0.37499997),
            (1.5625002, 0.75037587, 0.37499997),
            (1.5625002, 0.750376, 0.5),
            (1.4446021, 0.3367746, 0.40050507),
            (1.5254014, 0.625217, 0.5243553),
            (1.4767863, 0.36174792, 0.10701607),
            (1.4640422, 0.37684005, 0.2983256),
            (1.4834824, 0.41690552, 0.1961461),
        ],
        faces: &[
            (10, 25, 6),
            (29, 11, 27),
            (29, 0, 1),
            (2, 1, 0),
            (28, 11, 29),
            (25, 3, 11),
            (28, 4, 25),
            (2, 5, 1),
            (5, 4, 1),
            (25, 13, 3),
            (4, 6, 25),
            (5, 7, 4),
            (7, 6, 4),
            (26, 6, 7),
            (8, 26, 7),
            (26, 8, 9),
            (12, 11, 3),
            (11, 14, 27),
            (12, 15, 11),
            (15, 14, 11),
            (16, 15, 12),
            (29, 14, 17),
            (15, 18, 14),
            (18, 17, 14),
            (16, 19, 15),
            (19, 18, 15),
            (20, 19, 16),
            (17, 21, 0),
            (18, 22, 17),
            (22, 21, 17),
            (19, 26, 18),
            (26, 22, 18),
            (20, 26, 19),
            (22, 23, 21),
            (26, 24, 22),
            (24, 23, 22),
            (9, 24, 26),
            (13, 25, 10),
            (25, 11, 28),
            (27, 14, 29),
            (17, 0, 29),
            (6, 26, 20),
            (1, 28, 29),
            (4, 28, 1),
        ],
        outgoing: &[
            (1, &[0, 2, 4, 29, 5, 28], 0),
            (4, &[25, 5, 6, 7, 1, 28], 1),
            (7, &[5, 4, 8, 26, 6], 6),
            (11, &[29, 3, 25, 27, 12, 14, 15, 28], 3),
            (14, &[11, 27, 15, 17, 18, 29], 27),
            (15, &[12, 11, 14, 16, 18, 19], 18),
            (17, &[29, 14, 0, 18, 21, 22], 21),
            (18, &[15, 14, 17, 19, 22, 26], 22),
            (19, &[16, 15, 18, 20, 26], 20),
            (22, &[18, 17, 21, 26, 23, 24], 23),
            (25, &[11, 4, 3, 6, 28, 13, 10], 6),
            (26, &[7, 8, 9, 6, 19, 18, 22, 20, 24], 20),
            (27, &[11, 14, 29], 29),
            (28, &[29, 25, 1, 11, 4], 25),
            (29, &[27, 11, 1, 17, 28, 14, 0], 1),
        ],
        merge: (18, 28),
        protected: &[26, 27],
    },
    // ==== run 2: radius 1, +5 opened, 26 faces ====
    WeldCase {
        what: "reversed weld order, radius 1, +5 boundary halfedges",
        verts: &[
            (1.46697, 0.6317977, 0.25214934),
            (1.4374995, 0.75036615, 0.24963474),
            (1.4970202, 0.90742147, 0.3722189),
            (1.4972193, 0.78404593, 0.32863516),
            (1.4616873, 0.8957703, 0.3890407),
            (1.4495934, 0.82306826, 0.31933773),
            (1.4921471, 0.7405966, 0.10240269),
            (1.5004282, 0.9080494, 0.31580862),
            (1.5011928, 0.89758074, 0.2332556),
            (1.4946833, 0.76232123, 0.21551892),
            (1.5157567, 0.8215121, 0.1699916),
            (1.503637, 1.0320529, 0.3029821),
            (1.4968212, 1.030797, 0.4158027),
            (1.4968212, 1.030797, 0.4158027),
            (1.3786334, 1.1549265, 0.35936746),
            (1.6213622, 1.1549145, 0.35938823),
            (1.4226751, 1.1762068, 0.21613856),
            (1.5846666, 1.1656626, 0.24123849),
        ],
        faces: &[
            (8, 1, 10),
            (6, 1, 0),
            (10, 1, 6),
            (3, 10, 9),
            (1, 3, 0),
            (3, 4, 2),
            (5, 8, 4),
            (5, 3, 1),
            (11, 4, 8),
            (6, 0, 9),
            (4, 3, 5),
            (1, 8, 5),
            (11, 8, 7),
            (9, 0, 3),
            (10, 3, 8),
            (3, 2, 7),
            (7, 2, 11),
            (14, 13, 11),
            (11, 17, 16),
            (7, 8, 3),
            (2, 4, 12),
            (4, 11, 12),
            (16, 14, 11),
            (11, 13, 15),
            (9, 10, 6),
            (11, 15, 17),
        ],
        outgoing: &[
            (0, &[1, 3, 6, 9], 3),
            (1, &[10, 6, 0, 8, 3, 5], 0),
            (2, &[12, 11, 3, 4, 7], 12),
            (3, &[1, 4, 0, 9, 8, 7, 2, 10, 5], 0),
            (4, &[8, 5, 3, 11, 12, 2], 3),
            (5, &[1, 8, 4, 3], 1),
            (6, &[0, 1, 9, 10], 0),
            (7, &[3, 2, 11, 8], 3),
            (8, &[1, 4, 3, 11, 10, 7, 5], 3),
            (9, &[6, 0, 3, 10], 6),
            (10, &[6, 8, 3, 1, 9], 3),
            (11, &[8, 16, 4, 7, 13, 12, 14, 15, 17, 2], 13),
            (12, &[2, 4, 11], 4),
        ],
        merge: (9, 8),
        protected: &[13],
    },
    // ==== run 3: radius 1, +6 opened, 52 faces ====
    WeldCase {
        what: "reversed weld order, radius 1, +6 boundary halfedges",
        verts: &[
            (1.3093596, 0.37499982, 1.3093596),
            (1.3093596, 0.5, 1.3093596),
            (1.3093692, 0.6249998, 1.3093499),
            (1.4374998, 0.37499997, 0.75037587),
            (1.4374998, 0.5, 0.750376),
            (1.4367635, 0.37499925, 0.898358),
            (1.4367634, 0.5, 0.89835876),
            (1.5008514, 0.59192085, 0.8613802),
            (1.69064, 0.37499475, 1.3093587),
            (1.6902308, 0.4999436, 1.3089455),
            (1.6946573, 0.6250747, 1.3130403),
            (1.5077914, 0.27509907, 0.88640124),
            (1.5625002, 0.375, 0.7503758),
            (1.5632364, 0.37499958, 0.89835626),
            (1.5625002, 0.50000006, 0.750376),
            (1.5301834, 0.49542326, 0.90014625),
            (1.4945834, 0.5461893, 0.95348686),
            (1.3551339, 0.7116632, 1.2293802),
            (1.6364484, 0.7043385, 1.1903403),
            (1.5445898, 0.41506463, 1.1139808),
            (1.5055845, 0.33813843, 1.0295433),
            (1.4165065, 0.40624997, 1.040559),
            (1.4670306, 0.60281163, 1.1013482),
            (1.358845, 0.3296609, 1.2320743),
            (1.4883152, 0.5004578, 1.0455935),
            (1.3743107, 0.52626896, 1.1771944),
            (1.4142686, 0.1871017, 1.1966355),
            (1.6008643, 0.32967144, 1.182368),
            (1.6019799, 0.5948021, 1.174322),
            (1.4380281, 0.33481583, 1.1546514),
            (1.5594375, 0.6905519, 1.1465756),
            (1.6213715, 0.45300648, 1.1549394),
            (1.5692087, 0.20108268, 1.1939099),
            (1.4457461, 0.70516545, 1.1571027),
            (1.4061694, 0.4305424, 1.1659229),
            (1.3439966, 0.4296874, 1.2321427),
            (1.5257758, 0.4377287, 0.97197485),
        ],
        faces: &[
            (19, 20, 27),
            (27, 31, 19),
            (4, 6, 3),
            (6, 5, 3),
            (7, 6, 4),
            (21, 20, 5),
            (24, 21, 6),
            (20, 13, 11),
            (13, 15, 12),
            (15, 14, 12),
            (36, 15, 13),
            (15, 7, 14),
            (22, 25, 24),
            (30, 33, 28),
            (11, 5, 20),
            (23, 26, 29),
            (5, 6, 21),
            (34, 35, 29),
            (16, 6, 7),
            (13, 20, 36),
            (17, 25, 33),
            (25, 35, 34),
            (20, 29, 27),
            (36, 20, 24),
            (33, 25, 22),
            (22, 28, 33),
            (16, 15, 24),
            (19, 31, 24),
            (31, 28, 24),
            (20, 21, 29),
            (27, 29, 32),
            (21, 24, 25),
            (35, 1, 0),
            (25, 2, 1),
            (8, 9, 31),
            (9, 10, 28),
            (24, 28, 22),
            (29, 35, 23),
            (28, 10, 18),
            (24, 20, 19),
            (7, 15, 16),
            (26, 32, 29),
            (8, 31, 27),
            (34, 21, 25),
            (9, 28, 31),
            (24, 6, 16),
            (2, 25, 17),
            (29, 21, 34),
            (23, 35, 0),
            (25, 1, 35),
            (28, 18, 30),
            (24, 15, 36),
        ],
        outgoing: &[
            (6, &[4, 3, 5, 7, 21, 24, 16], 24),
            (15, &[13, 12, 14, 24, 7, 36, 16], 24),
            (16, &[7, 15, 24, 6], 7),
            (19, &[24, 20, 27, 31], 24),
            (20, &[11, 13, 5, 24, 21, 27, 29, 19, 36], 24),
            (21, &[5, 6, 20, 24, 25, 29, 34], 29),
            (22, &[33, 25, 24, 28], 33),
            (24, &[6, 19, 36, 15, 16, 22, 21, 20, 25, 31, 28], 19),
            (25, &[21, 2, 24, 1, 33, 34, 17, 35, 22], 24),
            (28, &[33, 9, 30, 10, 24, 18, 31, 22], 33),
            (29, &[21, 27, 20, 23, 32, 26, 34, 35], 23),
            (31, &[8, 24, 9, 27, 28, 19], 24),
            (34, &[25, 35, 29, 21], 25),
            (35, &[23, 25, 0, 29, 1, 34], 0),
            (36, &[24, 15, 13, 20], 24),
        ],
        merge: (34, 24),
        protected: &[22],
    },
    // ==== re-extracted run: radius 1, +4 opened, 33 faces ====
    WeldCase {
        what: "re-extracted reversed weld order, radius 1, +4 boundary halfedges",
        verts: &[
            (1.4374998, 0.3750002, 0.24962425),
            (1.5067351, 0.4945724, 0.25583982),
            (1.4933177, 0.62461275, 0.3489351),
            (1.4375, 0.37499997, 0.37499997),
            (1.4375, 0.5, 0.37499997),
            (1.4514543, 0.71893513, 0.39973378),
            (1.4940202, 0.35729223, 0.5034016),
            (1.5063298, 0.48580903, 0.48171258),
            (1.4479167, 0.6250626, 0.5416666),
            (1.4374998, 0.750376, 0.5),
            (1.4427084, 0.5625313, 0.4583333),
            (1.4961016, 0.7320268, 0.6284971),
            (1.5026203, 0.631256, 0.6254212),
            (1.5548742, 0.3993808, 0.3377111),
            (1.5625, 0.5, 0.37499997),
            (1.527909, 0.6874944, 0.42446756),
            (1.5041058, 0.768136, 0.24835946),
            (1.5331594, 0.74423194, 0.33999056),
            (1.5625002, 0.750376, 0.5),
            (1.5632365, 0.898358, 0.37499934),
            (1.498734, 0.8880255, 0.47460926),
            (1.5174797, 0.8773632, 0.6241799),
            (1.5414314, 0.596817, 0.5241195),
        ],
        faces: &[
            (1, 4, 0),
            (4, 3, 0),
            (9, 8, 5),
            (2, 4, 1),
            (16, 5, 2),
            (4, 7, 3),
            (7, 6, 3),
            (10, 2, 8),
            (10, 7, 4),
            (5, 8, 2),
            (8, 22, 7),
            (9, 11, 8),
            (22, 8, 12),
            (4, 2, 10),
            (8, 7, 10),
            (12, 8, 11),
            (6, 7, 13),
            (7, 14, 13),
            (2, 17, 16),
            (14, 2, 1),
            (7, 22, 14),
            (22, 2, 14),
            (15, 22, 18),
            (15, 17, 2),
            (11, 18, 22),
            (17, 19, 16),
            (18, 20, 17),
            (20, 19, 17),
            (11, 21, 18),
            (21, 20, 18),
            (11, 22, 12),
            (18, 17, 15),
            (2, 22, 15),
        ],
        outgoing: &[
            (2, &[16, 1, 4, 5, 8, 14, 22, 17, 10, 15], 5),
            (4, &[1, 0, 3, 2, 7, 10], 7),
            (7, &[4, 3, 8, 22, 6, 13, 14, 10], 22),
            (8, &[5, 2, 9, 7, 22, 11, 12, 10], 2),
            (9, &[5, 8, 20, 11, 21], 5),
            (10, &[8, 4, 2, 7], 4),
            (11, &[9, 21, 8, 22, 18, 12], 8),
            (12, &[11, 22, 8], 22),
            (14, &[13, 1, 7, 2, 22], 13),
            (15, &[22, 17, 18, 2], 2),
            (17, &[2, 16, 18, 19, 20, 15], 2),
            (18, &[15, 22, 17, 11, 20, 21], 20),
            (21, &[20, 9, 11, 18], 11),
            (22, &[8, 7, 14, 2, 18, 11, 12, 15], 8),
        ],
        merge: (15, 10),
        protected: &[12, 22],
    },
];

/// Weld merges whose standalone fixture cannot be closed, kept as evidence.
///
/// Both were printed by the extractor's `NO STANDALONE REPRO` fallback, which
/// emits the widest cut *without* minimising it when no radius reproduces the
/// defect. That branch is not a reproduction: it drops faces of the one-rings'
/// stars, so a *ring* vertex ends up on the cut and its rim becomes an open
/// chain. No boundary-respecting triangulation can consume an open chain (a
/// triangulation's boundary is always closed), so these entries can never be
/// "fixed" by a planner - they are fixture bugs. See
/// [`test_open_weld_chain_cases_are_cut_truncated`].
///
/// A genuine re-extraction of the same reversed soak, done with the rim-first
/// planner temporarily disabled and the capture radius raised, reproduces at
/// radius 1 and *is* closed by the planner; it is in [`WELD_CASES`].
const CUT_TRUNCATED_CASES: &[WeldCase] = &[
    // ==== run 1: radius 2, +4 opened, 36 faces ====
    WeldCase {
        what: "reversed weld order, radius 2, +4 boundary halfedges",
        verts: &[
            (1.3093596, 1.3093596, 0.37499982),
            (1.3093596, 1.3093596, 0.5),
            (1.4374995, 0.75036615, 0.24963474),
            (1.4374998, 0.75037587, 0.37499997),
            (1.4367635, 0.8983578, 0.37499928),
            (1.4371406, 1.1182438, 0.4349519),
            (1.6906404, 1.3093592, 0.5),
            (1.5625005, 0.750367, 0.24963427),
            (1.5625002, 0.75037587, 0.37499997),
            (1.5011928, 0.89758074, 0.2332556),
            (1.5632365, 0.898358, 0.37499934),
            (1.5079055, 0.8533567, 0.50018746),
            (1.5157567, 0.8215121, 0.1699916),
            (1.503637, 1.0320529, 0.3029821),
            (1.5040874, 1.030797, 0.4158027),
            (1.5040874, 1.030797, 0.4158027),
            (1.4268423, 1.1804459, 0.61133164),
            (1.5848209, 1.1674366, 0.6233738),
            (1.3786334, 1.1549265, 0.35936746),
            (1.3701937, 1.2056906, 0.45410115),
            (1.562401, 1.162938, 0.48437628),
            (1.6213622, 1.1549145, 0.35938823),
            (1.4226751, 1.1762068, 0.21613856),
            (1.5846666, 1.1656626, 0.24123849),
            (1.3439963, 1.2321428, 0.30448413),
            (1.6560037, 1.2321429, 0.4296868),
            (1.3439965, 1.2321426, 0.55468804),
            (1.6560034, 1.2321424, 0.5546873),
        ],
        faces: &[
            (9, 2, 12),
            (9, 4, 2),
            (4, 3, 2),
            (24, 0, 18),
            (13, 4, 9),
            (4, 11, 3),
            (14, 11, 4),
            (19, 20, 5),
            (12, 7, 9),
            (8, 10, 7),
            (10, 9, 7),
            (11, 10, 8),
            (15, 18, 5),
            (14, 13, 10),
            (18, 15, 13),
            (13, 23, 22),
            (9, 10, 13),
            (10, 11, 14),
            (25, 21, 20),
            (27, 20, 17),
            (4, 13, 14),
            (22, 18, 13),
            (13, 15, 21),
            (5, 18, 19),
            (20, 16, 17),
            (5, 20, 15),
            (19, 18, 0),
            (26, 19, 1),
            (16, 20, 19),
            (15, 20, 21),
            (13, 21, 23),
            (18, 22, 24),
            (20, 6, 25),
            (16, 19, 26),
            (0, 1, 19),
            (6, 20, 27),
        ],
        outgoing: &[
            (4, &[9, 2, 3, 11, 13, 14], 14),
            (5, &[20, 18, 15, 19], 19),
            (9, &[2, 4, 7, 10, 13, 12], 12),
            (10, &[8, 7, 9, 11, 13, 14], 14),
            (13, &[9, 22, 4, 10, 15, 14, 18, 21, 23], 14),
            (14, &[4, 10, 11, 13], 13),
            (15, &[18, 5, 20, 13, 21], 13),
            (18, &[0, 5, 13, 15, 22, 24, 19], 19),
            (19, &[5, 20, 0, 18, 16, 26, 1], 5),
            (20, &[5, 15, 19, 6, 16, 17, 21, 25, 27], 15),
        ],
        merge: (13, 5),
        protected: &[14],
    },
    // ==== run 4: radius 2, +4 opened, 36 faces ====
    WeldCase {
        what: "reversed weld order, radius 2, +4 boundary halfedges (mirror)",
        verts: &[
            (1.3093596, 1.3093596, 0.37499982),
            (1.3093596, 1.3093596, 0.5),
            (1.4374995, 0.75036615, 0.24963474),
            (1.4374998, 0.75037587, 0.37499997),
            (1.4367635, 0.8983578, 0.37499928),
            (1.4371406, 1.1182438, 0.4349519),
            (1.6906404, 1.3093592, 0.5),
            (1.5625005, 0.750367, 0.24963427),
            (1.5625002, 0.75037587, 0.37499997),
            (1.5011928, 0.89758074, 0.2332556),
            (1.5632365, 0.898358, 0.37499934),
            (1.5079055, 0.8533567, 0.50018746),
            (1.5157567, 0.8215121, 0.1699916),
            (1.503637, 1.0320529, 0.3029821),
            (1.5040874, 1.030797, 0.4158027),
            (1.5040874, 1.030797, 0.4158027),
            (1.4268423, 1.1804459, 0.61133164),
            (1.5848209, 1.1674366, 0.6233738),
            (1.3786334, 1.1549265, 0.35936746),
            (1.3701937, 1.2056906, 0.45410115),
            (1.562401, 1.162938, 0.48437628),
            (1.6213622, 1.1549145, 0.35938823),
            (1.4226751, 1.1762068, 0.21613856),
            (1.5846666, 1.1656626, 0.24123849),
            (1.3439963, 1.2321428, 0.30448413),
            (1.6560037, 1.2321429, 0.4296868),
            (1.3439965, 1.2321426, 0.55468804),
            (1.6560034, 1.2321424, 0.5546873),
        ],
        faces: &[
            (9, 2, 12),
            (9, 4, 2),
            (4, 3, 2),
            (24, 0, 18),
            (13, 4, 9),
            (4, 11, 3),
            (14, 11, 4),
            (19, 20, 5),
            (12, 7, 9),
            (8, 10, 7),
            (10, 9, 7),
            (11, 10, 8),
            (15, 18, 5),
            (14, 13, 10),
            (18, 15, 13),
            (13, 23, 22),
            (9, 10, 13),
            (10, 11, 14),
            (25, 21, 20),
            (27, 20, 17),
            (4, 13, 14),
            (22, 18, 13),
            (13, 15, 21),
            (5, 18, 19),
            (20, 16, 17),
            (5, 20, 15),
            (19, 18, 0),
            (26, 19, 1),
            (16, 20, 19),
            (15, 20, 21),
            (13, 21, 23),
            (18, 22, 24),
            (20, 6, 25),
            (16, 19, 26),
            (0, 1, 19),
            (6, 20, 27),
        ],
        outgoing: &[
            (4, &[9, 2, 3, 11, 13, 14], 14),
            (5, &[20, 18, 15, 19], 19),
            (9, &[2, 4, 7, 10, 13, 12], 12),
            (10, &[8, 7, 9, 11, 13, 14], 14),
            (13, &[9, 22, 4, 10, 15, 14, 18, 21, 23], 14),
            (14, &[4, 10, 11, 13], 13),
            (15, &[18, 5, 20, 13, 21], 13),
            (18, &[0, 5, 13, 15, 22, 24, 19], 19),
            (19, &[5, 20, 0, 18, 16, 26, 1], 5),
            (20, &[5, 15, 19, 6, 16, 17, 21, 25, 27], 15),
        ],
        merge: (5, 13),
        protected: &[14],
    },
];

/// Genuinely-open weld cases: entries whose standalone fixture *reproduces* and
/// that the band planner still fails to close.
///
/// Empty as of the rim-first planner: the two entries that used to be here
/// (`radius 1, +5` and `radius 1, +6`) are closed and live in [`WELD_CASES`],
/// the re-extracted reversed-soak case is closed too, and the only other
/// ex-members turned out to be unfaithful fixtures ([`CUT_TRUNCATED_CASES`]).
/// [`test_open_weld_cases_are_open`] keeps watching this list.
/// The gap=0.05 weld's first failure: the merge of two vertices that share two ring
/// vertices opens four boundary halfedges and breaks manifoldness (`WELD_CAPTURE=1`:
/// `MANIFOLD BROKE at merge VertexId(36v1)+VertexId(415v1) (boundary 0 -> 4)`, one
/// shared-ring pair count 2). Captured at radius 1, 34 vertices / 45 faces.
///
/// It matters beyond itself: every later boundary the weld leaves at this gap is an
/// extension of these four halfedges, so nothing downstream can close them without
/// sealing boundary it did not open.
///
/// The two sheets lie face to face on three triangles (10/11/14/15 on 28/29/30/31). In the
/// capture that patch ran into the cut along 10-14-15 and 28-30-31, so the cut's borders
/// were part of what went wrong, and `replay` skips edges at cut vertices. Vertices 34 and
/// 35 extend the two sheets past those edges, fanning around every patch vertex that was
/// on the cut, and bend away from each other so nothing new coincides: the patch ends
/// inside the region, and whatever the merge opens around it is measured. 36 vertices /
/// 53 faces.
const WELD_GAP005_MERGE: WeldCase = WeldCase {
    what: "gap=0.05 weld: two shared ring vertices, band leaves the region open",
    verts: &[
        (0.75, 0.875, -0.025),
        (0.8953125, 0.90777326, 0.0004969053),
        (0.75, 0.75, -0.025),
        (0.875, 0.75, -0.025),
        (0.75, 0.625, -0.025),
        (0.875, 0.625, -0.025),
        (1.0176777, 0.625, -0.01767767),
        (0.875, 0.5, -0.025),
        (1.0176777, 0.5, -0.01767767),
        (1.0176777, 0.375, -0.01767767),
        (1.025, 0.5, 0.125),
        (1.025, 0.625, 0.125),
        (1.025, 0.75, 0.125),
        (1.001013, 0.91350406, 0.06866117),
        (1.025, 0.625, 0.25),
        (1.025, 0.75, 0.25),
        (1.025, 0.875, 0.25),
        (1.1401674, 0.80129445, 0.004497938),
        (1.0267129, 0.7756472, 0.003700425),
        (1.175, 0.625, -0.025),
        (1.0323223, 0.625, -0.01767767),
        (1.3, 0.625, -0.025),
        (1.175, 0.5, -0.025),
        (1.0323223, 0.5, -0.01767767),
        (1.3, 0.5, -0.025),
        (1.175, 0.375, -0.025),
        (1.0323223, 0.375, -0.01767767),
        (1.3, 0.375, -0.025),
        (1.025, 0.5, 0.125),
        (1.025, 0.625, 0.125),
        (1.025, 0.625, 0.25),
        (1.025, 0.75, 0.25),
        (1.0268306, 0.87941945, 0.21875),
        (1.2200837, 0.7131472, -0.010251031),
        // Not captured: 34 extends sheet `a` past 10-14-15, 35 sheet `b` past 28-30-31.
        (1.075, 0.625, 0.35),
        (0.975, 0.625, 0.35),
    ],
    faces: &[
        (21, 19, 33),
        (1, 3, 0),
        (3, 2, 0),
        (18, 3, 1),
        (3, 5, 2),
        (5, 4, 2),
        (18, 6, 3),
        (6, 5, 3),
        (5, 7, 4),
        (6, 8, 5),
        (8, 7, 5),
        (8, 9, 7),
        (8, 10, 9),
        (6, 11, 8),
        (11, 10, 8),
        (18, 12, 6),
        (12, 11, 6),
        (11, 14, 10),
        (12, 15, 11),
        (15, 14, 11),
        (16, 15, 12),
        (32, 13, 12),
        (17, 19, 18),
        (19, 20, 18),
        (33, 19, 17),
        (19, 22, 20),
        (22, 23, 20),
        (21, 24, 19),
        (24, 22, 19),
        (22, 25, 23),
        (25, 26, 23),
        (24, 27, 22),
        (27, 25, 22),
        (28, 23, 26),
        (28, 29, 23),
        (29, 20, 23),
        (30, 29, 28),
        (29, 12, 20),
        (12, 18, 20),
        (30, 31, 29),
        (31, 12, 29),
        (31, 32, 12),
        (18, 13, 17),
        (13, 18, 1),
        (13, 16, 12),
        (9, 10, 34),
        (10, 14, 34),
        (14, 15, 34),
        (15, 16, 34),
        (35, 28, 26),
        (30, 28, 35),
        (31, 30, 35),
        (32, 31, 35),
    ],
    outgoing: &[
        (3, &[1, 0, 2, 18, 5, 6], 5),
        (5, &[3, 2, 4, 6, 7, 8], 7),
        (6, &[18, 3, 5, 8, 11, 12], 11),
        (8, &[6, 5, 7, 9, 10, 11], 10),
        (11, &[6, 8, 10, 12, 14, 15], 14),
        (12, &[18, 6, 11, 15, 16, 29, 20, 31, 32, 13], 32),
        (18, &[13, 1, 3, 6, 17, 19, 20, 12], 12),
        (19, &[17, 18, 20, 21, 22, 24, 33], 22),
        (20, &[19, 18, 22, 23, 29, 12], 18),
        (22, &[19, 20, 23, 24, 25, 27], 25),
        (23, &[22, 20, 25, 26, 28, 29], 20),
        (29, &[28, 23, 20, 30, 12, 31], 12),
    ],
    merge: (6, 20),
    protected: &[13],
};

const OPEN_WELD_CASES: &[WeldCase] = &[MERGE_GAP01_TRIANGLE];

/// [`WELD_GAP005_MERGE`] with the coincident-sheet zip run first, as a weld would.
///
/// The fixture's two sheets lie face to face on three triangles (captures 10/11/14/15 on
/// 28/29/30/31), which is what makes the merge alone open the region: its coincident fusion
/// joins 11 and 29, the triangles at 12 become a flap, and whichever path the merge takes
/// one of them goes and 15/31 are left apart. Zipped first, the merge gets two sheets that
/// already meet along the patch's edge and has to close only what its own removals open -
/// without the nets too.
#[test]
fn test_gap005_zip_then_merge_stays_closed() {
    get_tracing_subscriber();

    let case = &WELD_GAP005_MERGE;
    let (mut mesh_graph, vertex_ids) = case.build();
    let capture = |i: usize| vertex_ids[i];
    let boundary_before = boundary_edges(&mesh_graph);

    let zip = mesh_graph.zip_coincident_patches(vertex_ids.iter().copied());

    assert_eq!((zip.zipped_patches, zip.refused_patches), (1, 0));
    assert_eq!(zip.removed_faces.len(), 6);
    let mut fused = zip.fused.clone();
    fused.sort();
    let mut expected = vec![
        (capture(28), capture(10)),
        (capture(29), capture(11)),
        (capture(30), capture(14)),
        (capture(31), capture(15)),
    ];
    expected.sort();
    assert_eq!(fused, expected);

    // The patch ends inside the region, so the zip leaves the boundary exactly as it was, up
    // to the fusions.
    let survivor = |v: VertexId| {
        zip.fused
            .iter()
            .find(|(absorbed, _)| *absorbed == v)
            .map_or(v, |(_, s)| *s)
    };
    let boundary_zipped = boundary_edges(&mesh_graph);
    let before_renamed: std::collections::BTreeSet<_> = boundary_before
        .iter()
        .map(|&(a, b)| {
            let (a, b) = (survivor(a), survivor(b));
            (a.min(b), a.max(b))
        })
        .collect();
    assert_eq!(boundary_zipped, before_renamed);
    assert_manifold_region(&mesh_graph);

    let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
    let mut marked_vertices: HashSet<VertexId> = HashSet::default();
    for idx in case.protected {
        marked_vertices.insert(vertex_ids[*idx]);
    }
    mesh_graph.merge_vertices_one_rings(
        vertex_ids[case.merge.0],
        vertex_ids[case.merge.1],
        0.0096,
        &mut marked_halfedges,
        &mut marked_vertices,
    );
    drain_rim_taxonomy();

    let boundary_merged = boundary_edges(&mesh_graph);
    assert_eq!(
        boundary_merged,
        boundary_zipped,
        "the merge changed the boundary: opened {:?}, closed {:?}",
        boundary_merged
            .difference(&boundary_zipped)
            .collect::<Vec<_>>(),
        boundary_zipped
            .difference(&boundary_merged)
            .collect::<Vec<_>>(),
    );
    assert_manifold_region(&mesh_graph);
}

/// The extracted cases reproduce identically on every run - that is what makes
/// them usable as tests while the weld that produced them is still
/// nondeterministic. This guards the extraction, not the defect, so it passes
/// today and must keep passing after each case is fixed.
#[test]
fn test_weld_cases_are_deterministic() {
    get_tracing_subscriber();

    for case in WELD_CASES.iter().chain(CUT_TRUNCATED_CASES) {
        let mut outcomes = std::collections::BTreeSet::new();
        for _ in 0..100 {
            let (before, after, opened) = case.replay();
            outcomes.insert(format!("{before} -> {after} {opened:?}"));
        }

        assert_eq!(
            outcomes.len(),
            1,
            "case {:?} is not reproducible: 100 replays gave {} different outcomes: {outcomes:#?}",
            case.what,
            outcomes.len(),
        );
    }
}

/// Each captured case must leave both merged vertices interior - otherwise the
/// op sees a truncated one-ring the real mesh never had, and whatever it does is
/// about the cut, not about the defect.
#[test]
fn test_weld_cases_are_faithful() {
    get_tracing_subscriber();

    for case in WELD_CASES.iter().chain(CUT_TRUNCATED_CASES) {
        let (mesh_graph, vertex_ids) = case.build();
        let rim: std::collections::BTreeSet<VertexId> = boundary_edges(&mesh_graph)
            .iter()
            .flat_map(|(a, b)| [*a, *b])
            .collect();

        for idx in [case.merge.0, case.merge.1] {
            assert!(
                !rim.contains(&vertex_ids[idx]),
                "case {:?}: merged vertex {idx} sits on the cut, so its one-ring is truncated",
                case.what,
            );
        }
    }
}

/// None of [`WELD_CASES`] may open its region.
///
/// The case here: the band planner finished a full wrap around both rings and
/// left one triangle of the closing seam unfilled - the last planned face and
/// the first meet at a quad and only one of its two triangles was planned,
/// because the start cap reads `AlreadyConnected`, which is true of the edge but
/// not of the face. `close_triangular_holes` seals it.
#[test]
fn test_weld_cases_stay_watertight() {
    get_tracing_subscriber();

    let mut failures = Vec::new();

    for case in WELD_CASES {
        let (before, after, opened) = case.replay();
        drain_rim_taxonomy();
        if !opened.is_empty() {
            failures.push(format!(
                "  {:?}: boundary {before} -> {after}, opened interior edges {opened:?}",
                case.what
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} weld cases open their region:\n{}",
        failures.len(),
        WELD_CASES.len(),
        failures.join("\n"),
    );
}

/// The extracted open cases must still open: this pins the Phase 2 target list.
///
/// When a case starts closing, this test fails and the case moves to
/// [`WELD_CASES`] (with the taxonomy baseline updated). That failure is the
/// progress signal, not a regression.
/// Read-only: what each open merge case leaves, halfedge by halfedge.
#[test]
#[ignore]
fn diag_open_weld_cases() {
    get_tracing_subscriber();

    for case in OPEN_WELD_CASES {
        let (mut mesh_graph, vertex_ids) = case.build();
        let idx = |v: VertexId| {
            vertex_ids
                .iter()
                .position(|candidate| *candidate == v)
                .map(|i| i.to_string())
                .unwrap_or_else(|| format!("{v:?}"))
        };

        let before = boundary_edges(&mesh_graph);
        let mut marked_h: HashSet<HalfedgeId> = HashSet::default();
        let mut marked_v: HashSet<VertexId> = HashSet::default();
        for i in case.protected {
            marked_v.insert(vertex_ids[*i]);
        }

        mesh_graph.merge_vertices_one_rings(
            vertex_ids[case.merge.0],
            vertex_ids[case.merge.1],
            0.0096,
            &mut marked_h,
            &mut marked_v,
        );

        let after = boundary_edges(&mesh_graph);
        println!(
            "DIAG case {:?}: boundary {} -> {}",
            case.what,
            before.len(),
            after.len()
        );

        for (he_id, he) in mesh_graph.halfedges.iter() {
            if he.face.is_some() {
                continue;
            }

            let Some(start) = he.start_vertex(&mesh_graph) else {
                continue;
            };
            let key = (start.min(he.end_vertex), start.max(he.end_vertex));
            let out_new = |v: VertexId| {
                mesh_graph
                    .outgoing_halfedges
                    .get(v)
                    .map(|hes| {
                        hes.iter()
                            .filter(|h| {
                                mesh_graph.halfedges.get(**h).is_some_and(|x| {
                                    x.face.is_none()
                                        && x.start_vertex(&mesh_graph).is_some_and(|sv| {
                                            !before.contains(&(
                                                sv.min(x.end_vertex),
                                                sv.max(x.end_vertex),
                                            ))
                                        })
                                })
                            })
                            .count()
                    })
                    .unwrap_or(0)
            };

            println!(
                "DIAG he {he_id:?} {} -> {} new {} out_new {}",
                idx(start),
                idx(he.end_vertex),
                !before.contains(&key),
                out_new(start)
            );
        }
    }
}

#[test]
fn test_open_weld_cases_are_open() {
    get_tracing_subscriber();

    let mut failures = Vec::new();

    for case in OPEN_WELD_CASES {
        let (before, after, opened) = case.replay();
        drain_rim_taxonomy();
        if opened.is_empty() {
            failures.push(format!(
                "  {:?}: boundary {before} -> {after} - no longer opens; move it to WELD_CASES",
                case.what
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} open weld cases no longer open:\n{}",
        failures.len(),
        OPEN_WELD_CASES.len(),
        failures.join("\n"),
    );
}

#[test]
#[ignore]
fn diag_open_case() {
    get_tracing_subscriber();
    let idx: usize = std::env::var("CASE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let case = if std::env::var("OPEN").is_ok() {
        &CUT_TRUNCATED_CASES[idx]
    } else {
        &WELD_CASES[idx]
    };
    println!(
        "DIAG case {:?} merge {:?} protected {:?}",
        case.what, case.merge, case.protected
    );

    let (mut mesh_graph, vertex_ids) = case.build();
    println!("DIAG vertex_ids {vertex_ids:?}");
    let boundary_before = boundary_edges(&mesh_graph);
    let cut_rim: std::collections::BTreeSet<VertexId> =
        boundary_before.iter().flat_map(|(a, b)| [*a, *b]).collect();

    let capture_id = |v_id: VertexId| -> String {
        vertex_ids
            .iter()
            .position(|v| *v == v_id)
            .map(|i| i.to_string())
            .unwrap_or_else(|| format!("{v_id:?}"))
    };

    for (a, b) in &boundary_before {
        println!("DIAG cut edge ({}, {})", capture_id(*a), capture_id(*b));
    }

    let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
    let mut marked_vertices: HashSet<VertexId> = HashSet::default();
    for i in case.protected {
        marked_vertices.insert(vertex_ids[*i]);
    }
    mesh_graph.merge_vertices_one_rings(
        vertex_ids[case.merge.0],
        vertex_ids[case.merge.1],
        0.0096,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    let boundary_after = boundary_edges(&mesh_graph);
    println!(
        "DIAG boundary {} -> {}",
        boundary_before.len(),
        boundary_after.len()
    );
    for (a, b) in boundary_after.difference(&boundary_before) {
        let on_rim = cut_rim.contains(a) || cut_rim.contains(b);
        println!(
            "DIAG new boundary ({}, {})  on_cut_rim {on_rim}",
            capture_id(*a),
            capture_id(*b)
        );
    }

    for probe in super::rim_probe::drain() {
        println!("DIAG probe {}", probe.summary());
    }
}

/// Phase 0 baseline of the rim taxonomy, pinned so drift shows up as a test.
///
/// These are pre-net numbers: measured right after `add_planned_faces`, before
/// `weld_parallel_edges`/`close_triangular_holes` run. They are expected to
/// move when Phase 1/2 make the rebuild consume the rim structurally; when they
/// do, update this table - the point is that the change is explicit, not
/// silent. `(unconsumed, new_boundary, wire)`.
#[test]
fn test_rim_taxonomy_baseline() {
    get_tracing_subscriber();

    // (unconsumed, new_boundary, wire, skipped, parallel_before, parallel_after,
    //  end_unconsumed, end_new_boundary, end_wire, end_parallel, rim_bad). The
    //  pre-net half is mode-independent; the end half pins the shipping
    //  (nets-on) exit state and is skipped when the nets are disabled on
    //  purpose.
    let nets_enabled = super::merge_nets_enabled();
    let assert_case = |label: &str, expected: [usize; 11]| {
        let probes = super::rim_probe::drain();

        if label == FAST_PATH_CASE {
            assert_eq!(
                probes.len(),
                0,
                "{label}: expected no band probe on the fast path"
            );
            return;
        }

        assert_eq!(
            probes.len(),
            1,
            "{label}: expected exactly one band-merge probe, got {}",
            probes.len()
        );
        let probe = &probes[0];
        assert_eq!(
            (
                probe.unconsumed.len(),
                probe.new_boundary.len(),
                probe.wire.len(),
                probe.skipped_faces,
                probe.parallel_before,
                probe.parallel_edges,
            ),
            (
                expected[0],
                expected[1],
                expected[2],
                expected[3],
                expected[4],
                expected[5],
            ),
            "{label}: pre-net taxonomy drifted: {}",
            probe.summary()
        );
        if nets_enabled {
            assert_eq!(
                (
                    probe.end_unconsumed,
                    probe.end_new_boundary,
                    probe.end_wire.len(),
                    probe.end_parallel_edges,
                ),
                (expected[6], expected[7], expected[8], expected[9]),
                "{label}: end-of-op taxonomy drifted: {}",
                probe.summary()
            );
        }
        assert_eq!(
            probe.rim_bad,
            expected[10],
            "{label}: rim walk classification drifted: {}",
            probe.summary()
        );
    };

    let _ = merge_lens_region(false);
    // Degenerate synthetic variant: its repair leaves rim halves unconsumed, but
    // no wire and no parallel edges. It is only used for order-independence.
    // The star removal now enumerates all fans of a pinched vertex, so this
    // (degenerate) variant removes one more face and its rim is one half
    // smaller than before; the values below are the post-fix baseline.
    // The coincident fusion now maps a ring-2 vertex to the ring-1 vertex it was actually
    // fused into (it used to take the last coincident one), so the rings the planner sees are
    // the real ones and the pre-net rim is smaller still.
    assert_case("lens(reconstruct=false)", [0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0]);

    let _ = merge_lens_region(true);
    assert_case("lens(reconstruct=true)", [0; 11]);

    for case in WELD_CASES {
        let expected = match case.what {
            // The rim-first planner closes this case's pre-net hole (unconsumed
            // and new_boundary are 0). Its first split zipped a chord along an
            // edge that already joins the two rings outside the hole, which left
            // two parallel halfedges for the nets; the lobe now moves its split
            // until no chord duplicates an existing edge, so nothing is parallel
            // pre-net either.
            "band wraps the full ring and leaves one seam triangle" => [0; 11],
            // Closed by the rim-first planner: the strip removal opens a small
            // closed rim it consumes exactly.
            "reversed weld order, radius 1, +5 boundary halfedges" => [0; 11],
            "reversed weld order, radius 1, +6 boundary halfedges" => [0; 11],
            // Re-extracted from the reversed soak with the rim-first planner
            // disabled and the capture radius raised: a genuine, minimised
            // reproduction that this planner closes.
            "re-extracted reversed weld order, radius 1, +4 boundary halfedges" => [0; 11],
            // The gap=0.05 weld's first opening: a merge of two vertices sharing two ring
            // vertices, whose coincident fusion opened four boundary halfedges before any
            // band was planned. The op now closes them itself, so the case is a weld case
            // rather than an open one.
            "gap=0.05 weld: two shared ring vertices, band leaves the region open" => [0; 11],
            other => panic!("no pinned rim taxonomy for weld case {other:?}"),
        };
        let _ = case.replay();
        assert_case(case.what, expected);
    }

    for case in CUT_TRUNCATED_CASES {
        // Cut-truncated (see `test_open_weld_chain_cases_are_cut_truncated`):
        // the planner refuses because their rim is an open chain, so no band is
        // built and the whole rim is open before the repair passes.
        let expected = match case.what {
            "reversed weld order, radius 2, +4 boundary halfedges" => {
                [9, 0, 0, 0, 0, 0, 3, 1, 0, 0, 0]
            }
            "reversed weld order, radius 2, +4 boundary halfedges (mirror)" => {
                [9, 0, 0, 0, 0, 0, 3, 1, 0, 0, 0]
            }
            other => panic!("no pinned rim taxonomy for open weld case {other:?}"),
        };
        let _ = case.replay();
        assert_case(case.what, expected);
    }
}

/// Read-only measurement of the rim-first planner over the captured corpus.
///
/// The probe records what [`super::rim_planner`] would plan on the live rim of
/// each case, independent of whether the op used it. The table pins that, so a
/// structural change to the planner shows up explicitly; a failure is a prompt
/// to confirm the change, not a regression.
///
/// `(faces, consumed, rim_len, unhandled)`. A case with `consumed == rim_len`
/// is one whose rim the planner can plan in full - the prerequisite for wiring
/// it in.
#[test]
fn test_rim_planner_corpus_measurement() {
    get_tracing_subscriber();

    let assert_case = |label: &str, expected: [usize; 4]| {
        let probes = super::rim_probe::drain();

        if label == FAST_PATH_CASE {
            assert_eq!(
                probes.len(),
                0,
                "{label}: expected no band probe on the fast path"
            );
            return;
        }

        assert_eq!(
            probes.len(),
            1,
            "{label}: expected exactly one band-merge probe, got {}",
            probes.len()
        );
        let probe = &probes[0];
        assert_eq!(
            (
                probe.proto_faces,
                probe.proto_consumed,
                probe.rim.len(),
                probe.proto_unhandled.len(),
            ),
            (expected[0], expected[1], expected[2], expected[3]),
            "{label}: rim-plan measurement drifted: {}",
            probe.summary()
        );
        if probe.proto_consumed == probe.rim.len() {
            assert!(
                probe.proto_unhandled.is_empty(),
                "{label}: fully consumed rim must have no unhandled structures: {}",
                probe.summary()
            );
        }
    };

    let _ = merge_lens_region(false);
    assert_case("lens(reconstruct=false)", [6, 8, 8, 0]);

    let _ = merge_lens_region(true);
    assert_case("lens(reconstruct=true)", [7, 11, 11, 0]);

    for case in WELD_CASES {
        let expected = match case.what {
            // A pinch inside a single cyclic walk (vertex 30 repeats); the pinch
            // decomposition splits it into simple lobes and fans each.
            "band wraps the full ring and leaves one seam triangle" => [8, 12, 12, 0],
            // Closed by the rim-first planner; these measure the rim after the
            // strip removal the planner itself performs.
            "reversed weld order, radius 1, +5 boundary halfedges" => [2, 4, 4, 0],
            "reversed weld order, radius 1, +6 boundary halfedges" => [10, 14, 14, 0],
            "re-extracted reversed weld order, radius 1, +4 boundary halfedges" => [5, 9, 9, 0],
            FAST_PATH_CASE => [0; 4],
            other => panic!("no pinned rim-plan measurement for weld case {other:?}"),
        };
        let _ = case.replay();
        assert_case(case.what, expected);
    }

    for case in CUT_TRUNCATED_CASES {
        let expected = match case.what {
            // The chain ends at a pre-existing border of the cut region: only
            // the pinched single-side loop can be filled from rim data alone, so
            // the planner refuses and no band is built.
            "reversed weld order, radius 2, +4 boundary halfedges" => [2, 4, 9, 1],
            "reversed weld order, radius 2, +4 boundary halfedges (mirror)" => [2, 4, 9, 1],
            other => panic!("no pinned rim-plan measurement for open weld case {other:?}"),
        };
        let _ = case.replay();
        assert_case(case.what, expected);
    }
}

/// The [`CUT_TRUNCATED_CASES`] fixtures are cut-truncated, not closable.
///
/// Their rim is an **open chain**: `rim::next_rim_half` ends on a half that was
/// already boundary before the merge. A triangulation's boundary is always a
/// union of closed curves, so no plan that consumes only rim halves and adds
/// only interior edges can consume an open chain. The only way to "close" these
/// fixtures is to consume the pre-existing border halfedges the chain runs into,
/// which would seal a genuine open border - exactly what `close_triangular_holes`
/// is written to avoid.
///
/// The extraction's faithfulness check (`test_weld_cases_are_faithful`) requires
/// both *merged* vertices to be interior; these cases satisfy that, but a vertex
/// of their one-ring still sits on the cut, which is what opens the chain. A
/// wider re-extraction is needed before they can serve as Phase 2 targets.
#[test]
fn test_open_weld_chain_cases_are_cut_truncated() {
    get_tracing_subscriber();

    for case in CUT_TRUNCATED_CASES {
        let _ = case.replay();
        let probes = super::rim_probe::drain();
        assert_eq!(probes.len(), 1, "{}: one probe expected", case.what);
        let probe = &probes[0];

        let open_structures: Vec<&String> = probe
            .proto_structure_detail
            .iter()
            .filter(|detail| detail.starts_with("open"))
            .collect();

        assert_eq!(
            open_structures.len(),
            1,
            "{}: expected exactly one open chain in the rim: {:?}",
            case.what,
            probe.proto_structure_detail
        );
        assert!(
            probe
                .proto_unhandled
                .iter()
                .any(|unhandled| unhandled.starts_with("open chain")),
            "{}: the open chain must be reported unhandled: {}",
            case.what,
            probe.summary()
        );
        assert!(
            probe.proto_consumed < probe.rim.len(),
            "{}: a truncated rim cannot be fully consumed: {}",
            case.what,
            probe.summary()
        );
    }
}

/// Phase 0 diagnostic: print the rim taxonomy of the captured defect classes.
///
/// `cargo test --features gltf --lib diag_rim_taxonomy -- --ignored --nocapture`
#[test]
#[ignore]
fn diag_rim_taxonomy() {
    get_tracing_subscriber();

    for reconstruct in [false, true] {
        let _ = merge_lens_region(reconstruct);
        for probe in super::rim_probe::drain() {
            println!("RIM lens(reconstruct={reconstruct}) {}", probe.summary());
        }
    }

    for case in WELD_CASES {
        let _ = case.replay();
        for probe in super::rim_probe::drain() {
            println!("RIM weld {:?} {}", case.what, probe.summary());
        }
    }

    for case in CUT_TRUNCATED_CASES {
        let _ = case.replay();
        for probe in super::rim_probe::drain() {
            println!("RIM open weld {:?} {}", case.what, probe.summary());
            for line in &probe.rim_walk_detail {
                println!("WALK {:?} {line}", case.what);
            }
        }
    }
}

/// The coincident fusion's removals must show up in the merge result, or callers holding
/// those ids (the BVH, the selection) are never told.
///
/// The expectation is what the fusion itself removes, replayed on a clone: for every
/// position-coincident pair across the two rings, `merge_vertices` on the same mesh.
#[test]
fn test_merge_one_ring_fusion_reports_everything_it_removes() {
    get_tracing_subscriber();

    // Without the reconstructed seam, five vertices sit at the origin, so the merge fuses
    // coincident ring vertices before it plans anything.
    let (mut mesh, vertex_ids) = build_lens_region(false);
    let (v1, v2) = (vertex_ids[LENS_MERGE_V1], vertex_ids[LENS_MERGE_V2]);

    let ring = |mesh: &MeshGraph, v_id: VertexId| -> Vec<VertexId> {
        mesh.outgoing_halfedges[v_id]
            .iter()
            .map(|he_id| mesh.halfedges[*he_id].end_vertex)
            .collect()
    };
    let (ring1, ring2) = (ring(&mesh, v1), ring(&mesh, v2));

    let mut replay = mesh.clone();
    let mut expected_faces = Vec::new();
    let mut expected_halfedges = Vec::new();
    let mut expected_vertices = Vec::new();
    for &a in &ring1 {
        for &b in &ring2 {
            if a != b
                && replay.vertices.contains_key(a)
                && replay.vertices.contains_key(b)
                && replay.positions[a] == replay.positions[b]
            {
                let fused = replay.merge_vertices([a, b]);
                expected_faces.extend(fused.removed_faces);
                expected_halfedges.extend(fused.removed_halfedges);
                expected_vertices.extend(fused.removed_vertices);
            }
        }
    }
    assert!(
        !expected_faces.is_empty(),
        "fixture must exercise the coincident fusion"
    );

    let mut marked_halfedges: HashSet<HalfedgeId> = HashSet::default();
    let mut marked_vertices: HashSet<VertexId> = HashSet::default();
    marked_vertices.insert(vertex_ids[9]);
    let result = mesh.merge_vertices_one_rings(
        v1,
        v2,
        LENS_FLIP_THRESHOLD_SQR,
        &mut marked_halfedges,
        &mut marked_vertices,
    );
    drain_rim_taxonomy();

    for id in &expected_faces {
        assert!(
            result.removed_faces.contains(id),
            "fused face {id:?} not reported"
        );
    }
    for id in &expected_halfedges {
        assert!(
            result.removed_halfedges.contains(id),
            "fused halfedge {id:?} not reported"
        );
    }
    for id in &expected_vertices {
        assert!(
            result.removed_vertices.contains(id),
            "fused vertex {id:?} not reported"
        );
    }
}

/// The merge closer must not seal a border that was already open, even when the merge
/// deleted that border's edges and re-created them under new halfedge ids. An id-keyed
/// "opened" set takes the new ids for edges the merge opened and closes the triangle.
#[test]
fn test_merge_closer_keeps_a_border_recreated_under_new_ids() {
    get_tracing_subscriber();

    // A tetrahedron missing one face: a three-edge border that is meant to stay open.
    let mut mesh = MeshGraph::new();
    let a = mesh.add_vertex(vec3(0.0, 0.0, 0.0));
    let b = mesh.add_vertex(vec3(1.0, 0.0, 0.0));
    let c = mesh.add_vertex(vec3(0.0, 1.0, 0.0));
    let d = mesh.add_vertex(vec3(0.0, 0.0, 1.0));
    mesh.add_face_from_vertices(a, c, b).unwrap();
    let side = mesh.add_face_from_vertices(a, b, d).unwrap().face_id;
    mesh.add_face_from_vertices(b, c, d).unwrap();
    // (c, a, d) is the missing face.

    let pairs_at_entry = mesh.boundary_edge_pairs();
    let boundary_at_entry = super::rim::RegionBoundary::new(&mesh, &[a, b, c, d]);
    let ids_at_entry = super::rim::boundary_halves(&mesh);
    assert_eq!(pairs_at_entry.len(), 3);

    // Remove and re-add a neighbour: the border halfedges it shares are re-created.
    mesh.remove_face(side);
    mesh.add_face_from_vertices(a, b, d).unwrap();
    assert_eq!(
        mesh.boundary_edge_pairs(),
        pairs_at_entry,
        "same border, by vertex pair"
    );
    assert!(
        super::rim::boundary_halves(&mesh) != ids_at_entry,
        "precondition: the border halfedges have new ids"
    );

    let faces_before = mesh.faces.len();
    let (mut marked_halfedges, mut marked_vertices) = (HashSet::default(), HashSet::default());
    let mut record = MergeRecord::new(&mut marked_halfedges, &mut marked_vertices);
    mesh.close_merge_opened_holes(
        &[a, b, c, d],
        &[],
        &boundary_at_entry,
        &crate::ops::collapse::VertexRenames::default(),
        &mut record,
    );
    let result = record.result;

    assert_eq!(mesh.faces.len(), faces_before, "the open border was sealed");
    assert!(result.added_faces.is_empty());
}

/// A vertex merge renames the absorbed vertex, and with it every old border edge at that
/// vertex. The closer's snapshot has to be carried along, or those edges have vertex pairs it
/// never saw, look newly opened, and a border that was already open gets sealed.
#[test]
fn test_merge_closer_keeps_a_border_at_absorbed_vertices() {
    get_tracing_subscriber();

    // A square pyramid with no base: the four base edges are an old border. Two opposite
    // base corners get absorbed into loose vertices, which renames all four edges.
    let mut mesh = MeshGraph::new();
    let a = mesh.add_vertex(vec3(0.0, 0.0, 0.0));
    let b = mesh.add_vertex(vec3(1.0, 0.0, 0.0));
    let c = mesh.add_vertex(vec3(1.0, 1.0, 0.0));
    let d = mesh.add_vertex(vec3(0.0, 1.0, 0.0));
    let apex = mesh.add_vertex(vec3(0.5, 0.5, 1.0));
    for (x, y) in [(a, b), (b, c), (c, d), (d, a)] {
        mesh.add_face_from_vertices(x, apex, y).unwrap();
    }
    let mut loose = Vec::new();
    for i in 0..2 {
        let o = 10.0 * (i + 1) as f32;
        let s = mesh.add_vertex(vec3(o, 0.0, 0.0));
        let p = mesh.add_vertex(vec3(o + 1.0, 0.0, 0.0));
        let q = mesh.add_vertex(vec3(o, 1.0, 0.0));
        mesh.add_face_from_vertices(s, p, q).unwrap();
        loose.push(s);
    }
    assert_eq!(mesh.boundary_edge_pairs().len(), 4 + 6);

    let all_vertices: Vec<VertexId> = mesh.vertices.keys().collect();
    let boundary_at_entry = super::rim::RegionBoundary::new(&mesh, &all_vertices);
    let mut renames = crate::ops::collapse::VertexRenames::default();
    for (survivor, absorbed) in [(loose[0], a), (loose[1], c)] {
        let merged = mesh.merge_vertices([survivor, absorbed]);
        assert!(merged.removed_vertices.contains(&absorbed));
        renames.absorb(absorbed, survivor);
    }

    let faces_before = mesh.faces.len();
    let (mut marked_halfedges, mut marked_vertices) = (HashSet::default(), HashSet::default());
    let mut record = MergeRecord::new(&mut marked_halfedges, &mut marked_vertices);
    mesh.close_merge_opened_holes(
        &[loose[0], b, loose[1], d],
        &[],
        &boundary_at_entry,
        &renames,
        &mut record,
    );

    assert_eq!(mesh.faces.len(), faces_before, "the open border was sealed");
}

/// The merge closer empties a three-halfedge hole around a lone triangle rather than closing
/// it into a zero-volume pillow, and books the removal on the result.
#[test]
fn test_merge_closer_removes_a_lone_triangle() {
    let mut mesh = MeshGraph::new();
    let a = mesh.add_vertex(vec3(0.0, 0.0, 0.0));
    let b = mesh.add_vertex(vec3(1.0, 0.0, 0.0));
    let c = mesh.add_vertex(vec3(0.0, 1.0, 0.0));
    let face_id = mesh.add_face_from_vertices(a, b, c).unwrap().face_id;

    let (mut marked_halfedges, mut marked_vertices) = (HashSet::default(), HashSet::default());
    let mut record = MergeRecord::new(&mut marked_halfedges, &mut marked_vertices);
    mesh.close_merge_opened_holes(
        &[a, b, c],
        &[],
        // Nothing was a border at entry: the snapshot of an empty mesh over these vertices.
        &super::rim::RegionBoundary::new(&MeshGraph::new(), &[a, b, c]),
        &crate::ops::collapse::VertexRenames::default(),
        &mut record,
    );
    let result = record.result;

    assert_eq!(mesh.faces.len(), 0, "the lone triangle is gone, no pillow");
    assert!(result.added_faces.is_empty());
    assert_eq!(result.removed_faces, vec![face_id]);
    assert_eq!(result.removed_halfedges.len(), 6);
    assert_eq!(result.removed_vertices.len(), 3);
}

#[test]
fn test_vertex_renames() {
    use crate::ops::collapse::VertexRenames;

    let mut mesh = MeshGraph::new();
    let [a, b, c, d, e] = [(); 5].map(|_| mesh.add_vertex(Vec3::ZERO));
    let key = |x: VertexId, y: VertexId| if x < y { (x, y) } else { (y, x) };
    let snapshot: HashSet<(VertexId, VertexId)> =
        HashSet::from_iter([key(a, b), key(b, c), key(a, c), key(c, d)]);

    // Nothing recorded: the snapshot as it is.
    assert_eq!(VertexRenames::default().resolve(&snapshot), snapshot);

    // An absorb renames; the merged edge itself drops; a-c becomes b-c, which is there.
    let mut renames = VertexRenames::default();
    renames.absorb(a, b);
    assert_eq!(
        renames.resolve(&snapshot),
        HashSet::from_iter([key(b, c), key(c, d)])
    );

    // Absorbs chain: a went into b, b then into d.
    renames.absorb(b, d);
    assert_eq!(renames.resolve(&snapshot), HashSet::from_iter([key(d, c)]));

    // A split keeps the original and gives the new vertex all of its old edges.
    let mut renames = VertexRenames::default();
    renames.split(c, e);
    assert_eq!(
        renames.resolve(&snapshot),
        HashSet::from_iter([
            key(a, b),
            key(b, c),
            key(a, c),
            key(c, d),
            key(b, e),
            key(a, e),
            key(e, d),
        ])
    );

    // A vertex split off an absorbing survivor inherits what the survivor carries.
    let mut renames = VertexRenames::default();
    renames.absorb(d, c);
    renames.split(c, e);
    let resolved = renames.resolve(&snapshot);
    assert!(resolved.contains(&key(c, e)), "the old c-d edge, now c-e");
    assert!(resolved.contains(&key(a, e)) && resolved.contains(&key(b, e)));
}

/// `RimPlan::is_complete` decides whether the op builds the rim plan, so each of
/// its refusals is pinned on a rim it can be checked by hand: the three face-less halves
/// around a lone triangle `abc`.
#[test]
fn test_rim_plan_is_complete_refuses_what_does_not_close_the_rim() {
    use super::rim_planner::RimPlan;
    use super::{PlannedFace, PlannedFaceOrder::Single};

    let mut mesh = MeshGraph::new();
    let a = mesh.add_vertex(vec3(0.0, 0.0, 0.0));
    let b = mesh.add_vertex(vec3(1.0, 0.0, 0.0));
    let c = mesh.add_vertex(vec3(0.0, 1.0, 0.0));
    let d = mesh.add_vertex(vec3(0.3, 0.3, 1.0));
    let e = mesh.add_vertex(vec3(0.3, -1.0, 0.5));
    mesh.add_face_from_vertices(a, b, c).unwrap();

    let rim: HashSet<HalfedgeId> = super::rim::boundary_halves(&mesh);
    assert_eq!(rim.len(), 3);

    let plan = |faces: &[(VertexId, VertexId, VertexId)]| RimPlan {
        faces: faces
            .iter()
            .map(|&(v1, v2, v3)| PlannedFace::new(v1, v2, v3, Single))
            .collect(),
        unhandled: Vec::new(),
    };

    // A tetrahedron over the rim: every rim half consumed, every other edge paired.
    assert!(plan(&[(a, c, d), (c, b, d), (b, a, d)]).is_complete(&mesh, &rim));

    // `abc` a second time: its directed edges already carry the existing face.
    assert!(!plan(&[(a, b, c)]).is_complete(&mesh, &rim));

    // Every rim half consumed, but `d -> a` and `b -> d`, `a -> e` and `e -> b` have no
    // partner in the plan and are not rim halves: the plan would open boundary of its own.
    assert!(!plan(&[(a, c, d), (c, b, d), (b, a, e)]).is_complete(&mesh, &rim));
}

/// A caller keeps its marked sets across many merges (freestyle's protected sets), so they
/// arrive holding ids the merge is about to remove. None of them may come back dead, and the
/// result must never report an id as both added and removed.
#[test]
fn test_merge_keeps_long_lived_marked_sets_live() {
    get_tracing_subscriber();

    let check = |label: &str, mut mesh: MeshGraph, merge: (VertexId, VertexId), flip: f32| {
        let mut marked_halfedges: HashSet<HalfedgeId> = mesh.halfedges.keys().collect();
        let mut marked_vertices: HashSet<VertexId> = mesh.vertices.keys().collect();

        let result = mesh.merge_vertices_one_rings(
            merge.0,
            merge.1,
            flip,
            &mut marked_halfedges,
            &mut marked_vertices,
        );
        drain_rim_taxonomy();

        assert!(
            !result.removed_halfedges.is_empty(),
            "{label}: the merge removed nothing - test is vacuous"
        );
        for he_id in &marked_halfedges {
            assert!(
                mesh.halfedges.contains_key(*he_id),
                "{label}: marked halfedge {he_id:?} is dead"
            );
        }
        for v_id in &marked_vertices {
            assert!(
                mesh.vertices.contains_key(*v_id),
                "{label}: marked vertex {v_id:?} is dead"
            );
        }
        for he_id in &result.added_halfedges {
            assert!(
                !result.removed_halfedges.contains(he_id),
                "{label}: halfedge {he_id:?} is reported as both added and removed"
            );
        }
        for face_id in &result.added_faces {
            assert!(
                !result.removed_faces.contains(face_id),
                "{label}: face {face_id:?} is reported as both added and removed"
            );
        }
    };

    for case in WELD_CASES.iter().chain(CUT_TRUNCATED_CASES) {
        let (mesh, vertex_ids) = case.build();
        check(
            case.what,
            mesh,
            (vertex_ids[case.merge.0], vertex_ids[case.merge.1]),
            // The weld's `SculptParams::min_edge_length_squared`, as in `WeldCase::replay`.
            0.0096,
        );
    }

    let (mesh, vertex_ids) = build_lens_region(true);
    check(
        "lens",
        mesh,
        (vertex_ids[LENS_MERGE_V1], vertex_ids[LENS_MERGE_V2]),
        LENS_FLIP_THRESHOLD_SQR,
    );
}

/// The parallel-edge weld drops orphan halves, and an orphan is often the free half a
/// planned face of the same merge just added: it must leave `added_halfedges` rather than
/// be listed as removed too, and must leave the marked set.
#[test]
fn test_weld_parallel_edges_drops_its_own_orphans_cleanly() {
    let mut mesh = MeshGraph::new();
    let a = mesh.add_vertex(vec3(0.0, 0.0, 0.0));
    let b = mesh.add_vertex(vec3(1.0, 0.0, 0.0));
    let c = mesh.add_vertex(vec3(0.5, 1.0, 0.0));
    let d = mesh.add_vertex(vec3(0.5, -1.0, 0.0));

    mesh.add_face_from_vertices(a, b, c).unwrap();
    let old_orphan = mesh
        .halfedges_from_to(b, a)
        .into_iter()
        .next()
        .expect("the face's own b -> a twin");

    // A second, parallel a-b edge, whose b -> a half carries the face on the other side.
    let parallel = mesh.add_edge(b, a).unwrap();
    let a_d = mesh.add_edge(a, d).unwrap();
    let d_b = mesh.add_edge(d, b).unwrap();
    mesh.add_face(
        parallel.start_to_end_he_id,
        a_d.start_to_end_he_id,
        d_b.start_to_end_he_id,
    );
    let new_orphan = parallel.twin_he_id;

    let mut marked_halfedges = HashSet::from_iter([new_orphan, old_orphan]);
    let mut marked_vertices = HashSet::default();
    let mut record = MergeRecord::new(&mut marked_halfedges, &mut marked_vertices);
    record.result.added_halfedges.push(new_orphan);

    mesh.weld_parallel_edges([a, b], &mut record);
    let result = record.result;

    assert!(!mesh.halfedges.contains_key(new_orphan));
    assert!(!mesh.halfedges.contains_key(old_orphan));
    assert!(result.added_halfedges.is_empty());
    assert_eq!(result.removed_halfedges, vec![old_orphan]);
    assert!(marked_halfedges.is_empty());
}
