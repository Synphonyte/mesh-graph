use std::borrow::Borrow;

use glam::{Quat, Vec2, Vec3};

pub fn vec3_array(v: impl Borrow<Vec3>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, v.borrow().z]
}

pub fn vec2_array(v: impl Borrow<Vec2>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, 0.0]
}

pub fn quat_array(q: impl Borrow<Quat>) -> [f32; 4] {
    [q.borrow().x, q.borrow().y, q.borrow().z, q.borrow().w]
}
