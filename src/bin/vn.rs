use mesh_graph::VertexId;

pub fn main() {
    use mesh_graph::integrations::gltf;

    let mut meshgraph = gltf::load("src/ops/cleanup/test/merge_common_one_ring.glb").unwrap();

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

    let _result = meshgraph.merge_vertices_one_rings(v_top_id, v_bottom_id, 1.0);
}
