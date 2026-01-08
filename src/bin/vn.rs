pub fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .with_line_number(true)
        .init();

    let mut meshgraph = MeshGraph::new();

    let center_v_id = meshgraph.insert_vertex(Vec3::new(0.0, 0.0, 1.0));

    let v1_id = meshgraph.insert_vertex(Vec3::new(-0.2, 0.0, 0.0));
    let (he_c_1_id, _) = meshgraph.insert_or_get_edge(center_v_id, v1_id);

    let v1p_id = meshgraph.insert_vertex(Vec3::new(-0.2, 0.0, 0.0));
    let (he_c_1p_id, he_1p_c_id) = meshgraph.insert_or_get_edge(center_v_id, v1p_id);

    let v2_id = meshgraph.insert_vertex(Vec3::new(-1.0, 1.0, 0.0));
    let (he_c_2_id, _) = meshgraph.insert_or_get_edge(center_v_id, v2_id);

    let v3_id = meshgraph.insert_vertex(Vec3::new(-1.0, -1.0, 0.0));
    let (he_c_3_id, _) = meshgraph.insert_or_get_edge(center_v_id, v3_id);

    let v4_id = meshgraph.insert_vertex(Vec3::new(0.2, 0.0, 0.0));
    let (he_c_4_id, _) = meshgraph.insert_or_get_edge(center_v_id, v4_id);

    let v4p_id = meshgraph.insert_vertex(Vec3::new(0.2, 0.0, 0.0));
    let (he_c_4p_id, he_4p_c_id) = meshgraph.insert_or_get_edge(center_v_id, v4p_id);

    let v5_id = meshgraph.insert_vertex(Vec3::new(1.0, -1.0, 0.0));
    let (he_c_5_id, _) = meshgraph.insert_or_get_edge(center_v_id, v5_id);

    let v6_id = meshgraph.insert_vertex(Vec3::new(1.0, 1.0, 0.0));
    let (he_c_6_id, _) = meshgraph.insert_or_get_edge(center_v_id, v6_id);

    meshgraph
        .create_face_from_halfedges(he_c_1_id, he_c_2_id)
        .unwrap();
    meshgraph
        .create_face_from_halfedges(he_c_2_id, he_c_3_id)
        .unwrap();
    meshgraph
        .create_face_from_halfedges(he_c_3_id, he_c_1p_id)
        .unwrap();

    meshgraph
        .create_face_from_halfedges(he_c_1p_id, he_c_4p_id)
        .unwrap();

    meshgraph
        .create_face_from_halfedges(he_c_4p_id, he_c_5_id)
        .unwrap();
    meshgraph
        .create_face_from_halfedges(he_c_5_id, he_c_6_id)
        .unwrap();
    meshgraph
        .create_face_from_halfedges(he_c_6_id, he_c_4_id)
        .unwrap();

    meshgraph
        .create_face_from_halfedges(he_c_1_id, he_c_4_id)
        .unwrap();

    meshgraph.halfedges[he_c_1p_id].end_vertex = v1_id;
    meshgraph.halfedges[he_c_4p_id].end_vertex = v4_id;

    // already created from face above
    let (he_1p_4p_id, he_4p_1p_id) = meshgraph.insert_or_get_edge(v1p_id, v4p_id);
    meshgraph.halfedges[he_1p_4p_id].end_vertex = v4_id;
    meshgraph.halfedges[he_4p_1p_id].end_vertex = v1_id;

    let (he_3_1p_id, he_1p_3_id) = meshgraph.insert_or_get_edge(v3_id, v1p_id);
    meshgraph.halfedges[he_3_1p_id].end_vertex = v1_id;

    let (he_4p_5_id, he_5_4p_id) = meshgraph.insert_or_get_edge(v4p_id, v5_id);
    meshgraph.halfedges[he_5_4p_id].end_vertex = v4_id;

    meshgraph.outgoing_halfedges[v1_id].push(he_1p_4p_id);
    meshgraph.outgoing_halfedges[v1_id].push(he_1p_c_id);
    meshgraph.outgoing_halfedges[v1_id].push(he_1p_3_id);
    meshgraph.outgoing_halfedges[v4_id].push(he_4p_1p_id);
    meshgraph.outgoing_halfedges[v4_id].push(he_4p_c_id);
    meshgraph.outgoing_halfedges[v4_id].push(he_4p_5_id);

    meshgraph.delete_only_vertex(v1p_id);
    meshgraph.delete_only_vertex(v4p_id);

    // #[cfg(feature = "rerun")]
    // meshgraph.log_rerun();

    let mut removed_vertices = vec![];
    let mut removed_halfedges = vec![];
    let mut removed_faces = vec![];

    meshgraph.remove_degenerate_faces(
        center_v_id,
        &mut removed_vertices,
        &mut removed_halfedges,
        &mut removed_faces,
    );

    #[cfg(feature = "rerun")]
    {
        info!("finished");
        meshgraph.log_rerun();
        crate::RR.flush_blocking().unwrap();
    }
}
