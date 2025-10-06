#[cfg(feature = "rerun")]
use std::borrow::Borrow;

#[cfg(feature = "rerun")]
use glam::{Quat, Vec2, Vec3};

#[cfg(feature = "rerun")]
pub fn vec3_array(v: impl Borrow<Vec3>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, v.borrow().z]
}

#[cfg(feature = "rerun")]
pub fn vec2_array(v: impl Borrow<Vec2>) -> [f32; 3] {
    [v.borrow().x, v.borrow().y, 0.0]
}

#[cfg(feature = "rerun")]
pub fn quat_array(q: impl Borrow<Quat>) -> [f32; 4] {
    [q.borrow().x, q.borrow().y, q.borrow().z, q.borrow().w]
}

#[macro_export]
macro_rules! error_none {
    ($msg:literal) => {
        || {
            tracing::error!($msg);
            None
        }
    };
}

macro_rules! unwrap_or_return {
    ($code:expr, $error:expr, $ret:expr) => {
        match $code {
            Some(value) => value,
            None => {
                tracing::error!($error);
                return $ret;
            }
        }
    };
    ($code:expr, $error:expr) => {
        match $code {
            Some(value) => value,
            None => {
                tracing::error!($error);
                return;
            }
        }
    };
}

pub(crate) use unwrap_or_return;
