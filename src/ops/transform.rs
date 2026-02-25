use glam::Quat;

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
}
