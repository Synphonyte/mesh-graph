use glam::Vec3;
use serde::Deserialize;
use slotmap::{SecondaryMap, SlotMap};

use crate::{Face, FaceId, Halfedge, HalfedgeId, MeshGraph, Vertex, VertexId};

#[derive(Deserialize)]
pub struct MeshGraphIntermediate {
    pub vertices: SlotMap<VertexId, Vertex>,
    pub halfedges: SlotMap<HalfedgeId, Halfedge>,
    pub faces: SlotMap<FaceId, Face>,

    pub positions: SecondaryMap<VertexId, Vec3>,
    pub vertex_normals: Option<SecondaryMap<VertexId, Vec3>>,
}

impl MeshGraph {
    /// Writes the complete mesh state as JSON (via the `instrumentation` feature),
    /// so a corrupted state can be inspected or resumed from with [`Self::load_state`].
    ///
    /// The derived caches (`bvh`, `outgoing_halfedges`, ...) are not serialized;
    /// they are rebuilt when the state is loaded.
    #[cfg(feature = "instrumentation")]
    pub fn save_state(&self, path: impl AsRef<std::path::Path>) -> Result<(), anyhow::Error> {
        let file = std::fs::File::create(path)?;
        serde_json::to_writer(file, self)?;
        Ok(())
    }

    /// Reconstructs a mesh from a state file written by [`Self::save_state`] (or the
    /// automatic state-history dump). The BVH and outgoing halfedge lists are
    /// rebuilt from the topology.
    #[cfg(feature = "instrumentation")]
    pub fn load_state(path: impl AsRef<std::path::Path>) -> Result<MeshGraph, anyhow::Error> {
        let file = std::fs::File::open(path)?;
        Ok(serde_json::from_reader(file)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::vec3;

    #[test]
    #[cfg(feature = "instrumentation")]
    fn test_save_load_state_round_trip() {
        let mut mesh_graph = MeshGraph::new();
        // Two adjacent triangles so halfedges get twins and faces exist.
        mesh_graph.add_face_from_positions(
            vec3(0.0, 0.0, 0.0),
            vec3(1.0, 0.0, 0.0),
            vec3(0.0, 1.0, 0.0),
        );
        mesh_graph
            .add_face_from_halfedge_and_position(
                mesh_graph
                    .vertices
                    .values()
                    .next()
                    .unwrap()
                    .outgoing_halfedge
                    .unwrap(),
                vec3(1.0, 1.0, 0.0),
            )
            .unwrap();

        let path = std::env::temp_dir().join("mesh_graph_state_round_trip_test.json");
        mesh_graph.save_state(&path).unwrap();
        let loaded = MeshGraph::load_state(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(loaded.vertices.len(), mesh_graph.vertices.len());
        assert_eq!(loaded.halfedges.len(), mesh_graph.halfedges.len());
        assert_eq!(loaded.faces.len(), mesh_graph.faces.len());

        // Topology must match exactly (caches like `outgoing_halfedges` are rebuilt).
        for (he_id, he) in &mesh_graph.halfedges {
            let loaded_he = loaded.halfedges.get(he_id).unwrap();
            assert_eq!(loaded_he.end_vertex, he.end_vertex);
            assert_eq!(loaded_he.next, he.next);
            assert_eq!(loaded_he.twin, he.twin);
            assert_eq!(loaded_he.face, he.face);
        }
        for (f_id, face) in &mesh_graph.faces {
            let loaded_face = loaded.faces.get(f_id).unwrap();
            assert_eq!(loaded_face.halfedge, face.halfedge);
        }
        for (v_id, pos) in &mesh_graph.positions {
            assert_eq!(loaded.positions.get(v_id).unwrap(), pos);
        }
    }

    #[test]
    #[cfg(feature = "instrumentation")]
    fn test_state_history_ring_cap() {
        // The ring is gated on the hunt env var; enable it for this test only.
        unsafe {
            std::env::set_var("MESH_GRAPH_DANGLING_CHECK", "1");
        }

        let mut mesh_graph = MeshGraph::new();
        mesh_graph.add_face_from_positions(
            vec3(0.0, 0.0, 0.0),
            vec3(1.0, 0.0, 0.0),
            vec3(0.0, 1.0, 0.0),
        );

        for _ in 0..15 {
            crate::state_history_push(&mesh_graph, "test_op");
        }

        let len = crate::STATE_RING.lock().unwrap().len();
        assert_eq!(len, crate::STATE_RING_DEFAULT_CAP);

        unsafe {
            std::env::remove_var("MESH_GRAPH_DANGLING_CHECK");
        }
    }
}

impl From<MeshGraphIntermediate> for MeshGraph {
    fn from(value: MeshGraphIntermediate) -> Self {
        let mut mesh_graph = Self {
            bvh: Default::default(),
            bvh_workspace: Default::default(),
            index_to_face_id: Default::default(),
            next_index: 0,
            vertices: value.vertices,
            halfedges: value.halfedges,
            faces: value.faces,
            positions: value.positions,
            vertex_normals: value.vertex_normals,
            outgoing_halfedges: Default::default(),
        };

        for (id, face) in &mut mesh_graph.faces {
            face.index = mesh_graph.next_index;
            mesh_graph.next_index += 1;

            mesh_graph.index_to_face_id.insert(face.index, id);
        }

        mesh_graph.rebuild_bvh();
        mesh_graph.rebuild_outgoing_halfedges();

        mesh_graph
    }
}
