use delaunay::{
    core::{DataType, Tds},
    prelude::Coordinate,
};
use glam::Vec3;
use hashbrown::HashMap;

use crate::MeshGraph;

impl<U, V> From<Tds<f32, U, V, 3>> for MeshGraph
where
    U: DataType,
    V: DataType,
{
    fn from(value: Tds<f32, U, V, 3>) -> Self {
        let mut vertex_id_to_index = HashMap::new();
        let mut vertex_positions = Vec::with_capacity(value.vertices().len());

        for vertex in value.vertices().values() {
            vertex_id_to_index.insert(vertex.uuid(), vertex_positions.len());

            let pt = vertex.point().to_array();
            vertex_positions.push(Vec3::new(pt[0], pt[1], pt[2]));
        }

        let mut face_count = HashMap::<[usize; 3], usize>::new();
        let mut face_tetra_centroid = HashMap::<[usize; 3], Vec3>::new();

        for cell in value.cells().values() {
            let mut vertex_ids = cell
                .vertex_uuids()
                .iter()
                .map(|uuid| vertex_id_to_index[uuid])
                .collect::<Vec<_>>();

            vertex_ids.sort();

            let centroid = vertex_ids
                .iter()
                .map(|idx| vertex_positions[*idx])
                .sum::<Vec3>()
                * 0.25;

            let faces = [
                [vertex_ids[0], vertex_ids[1], vertex_ids[2]],
                [vertex_ids[0], vertex_ids[1], vertex_ids[3]],
                [vertex_ids[0], vertex_ids[2], vertex_ids[3]],
                [vertex_ids[1], vertex_ids[2], vertex_ids[3]],
            ];

            for face in faces {
                *face_count.entry(face).or_default() += 1;
                face_tetra_centroid.insert(face, centroid);
            }
        }

        let indices = face_count
            .into_iter()
            .filter_map(|([ia, ib, ic], count)| {
                if count == 1 {
                    let centroid = face_tetra_centroid[&[ia, ib, ic]];

                    let a = vertex_positions[ia];
                    let b = vertex_positions[ib];
                    let c = vertex_positions[ic];

                    let norm = (c - a).cross(c - b);

                    let indices = if norm.dot(centroid - a) < 0.0 {
                        // order is wrong => flip
                        [ic, ib, ia]
                    } else {
                        [ia, ib, ic]
                    };

                    Some(indices)
                } else {
                    None
                }
            })
            .flatten()
            .collect::<Vec<_>>();

        println!("pos: {vertex_positions:#?}");
        println!("idx: {indices:#?}");

        Self::indexed_triangles(&vertex_positions, &indices)
    }
}

#[cfg(test)]
mod test {
    use delaunay::prelude::generate_random_triangulation;

    use super::*;

    #[test]
    fn test_from_tds_to_mesh_graph() {
        let tds: Tds<f32, (), (), 3> = match generate_random_triangulation(
            10,            // Number of points
            (-10.0, 10.0), // Coordinate bounds
            None,          // No vertex data
            Some(42),      // Fixed seed for reproducibility
        ) {
            Ok(triangulation) => triangulation,
            Err(e) => {
                panic!("✗ Failed to create triangulation: {e}");
            }
        };

        let mesh_graph = MeshGraph::from(tds);

        mesh_graph.log_rerun();
        crate::RR.flush_blocking();
    }
}
