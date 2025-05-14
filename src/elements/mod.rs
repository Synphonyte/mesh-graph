mod face;
mod halfedge;
mod vertex;

pub use face::*;
pub use halfedge::*;
pub use vertex::*;

use slotmap::new_key_type;

new_key_type! { pub struct VertexId; }
new_key_type! { pub struct HalfedgeId; }
new_key_type! { pub struct FaceId; }
