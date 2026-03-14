use glam::{Mat4, Quat};

use crate::MeshGraph;

impl MeshGraph {
    /// Apply a quaternion rotation to the mesh graph (positions and normals).
    ///
    /// After this you should probably call `rebuild_bvh` to update the bounding volume hierarchy.
    pub fn apply_quat(&mut self, quat: Quat) {
        for pos in self.positions.values_mut() {
            *pos = quat * *pos;
        }

        if let Some(normals) = self.vertex_normals.as_mut() {
            for normal in normals.values_mut() {
                *normal = quat * *normal;
            }
        }
    }

    /// Apply a projection to the mesh graph (positions and normals).
    ///
    /// After this you should probably call `rebuild_bvh` to update the bounding volume hierarchy.
    pub fn apply_projection(&mut self, projection: Mat4) {
        for pos in self.positions.values_mut() {
            *pos = projection.project_point3(*pos);
        }

        if self.vertex_normals.is_some() {
            self.compute_vertex_normals();
        }
    }

    /// Apply a transform matrix to the mesh graph (positions and normals).
    ///
    /// After this you should probably call `rebuild_bvh` to update the bounding volume hierarchy.
    pub fn apply_transform(&mut self, transform: Mat4) {
        for pos in self.positions.values_mut() {
            *pos = transform.transform_point3(*pos);
        }

        if let Some(normals) = self.vertex_normals.as_mut() {
            for normal in normals.values_mut() {
                *normal = transform.transform_vector3(*normal);
            }
        }
    }
}
