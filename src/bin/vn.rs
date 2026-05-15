use hashbrown::HashSet;
use mesh_graph::VertexId;

pub fn main() {
    use mesh_graph::integrations::gltf;

    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_line_number(true)
        .pretty()
        .init();

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

    let _ = meshgraph.merge_vertices_one_rings(
        v_top_id,
        v_bottom_id,
        0.01,
        &mut marked_halfedges,
        &mut marked_vertices,
    );

    #[cfg(feature = "rerun")]
    {
        meshgraph.log_rerun();
        mesh_graph::RR.flush_blocking().unwrap();
    }
}
