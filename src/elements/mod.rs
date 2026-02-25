mod face;
mod halfedge;
mod vertex;

pub use face::*;
pub use halfedge::*;
pub use vertex::*;

use slotmap::{Key, KeyData, new_key_type};

new_key_type! { pub struct VertexId; }
new_key_type! { pub struct HalfedgeId; }
new_key_type! { pub struct FaceId; }

impl From<u128> for VertexId {
    fn from(value: u128) -> Self {
        KeyData::from_ffi(value as u64).into()
    }
}

impl From<VertexId> for u128 {
    fn from(value: VertexId) -> Self {
        value.data().as_ffi() as u128
    }
}
