use std::borrow::Borrow;

use glam::Vec3;

pub fn vec3_array(v: impl Borrow<Vec3>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, v.borrow().z]
}
