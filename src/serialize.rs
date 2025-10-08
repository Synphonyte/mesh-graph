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
        };

        for (id, face) in &mut mesh_graph.faces {
            face.index = mesh_graph.next_index;
            mesh_graph.next_index += 1;

            mesh_graph.index_to_face_id.insert(face.index, id);
        }

        mesh_graph.rebuild_bvh();

        mesh_graph
    }
}
