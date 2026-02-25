mod add;
mod cleanup;
mod collapse;
mod edit;
mod merge_one_ring;
mod query;
mod remove;
mod subdivide;
mod transform;

pub use add::*;
use hashbrown::HashMap;
pub use merge_one_ring::*;

use crate::{HalfedgeId, MeshGraph, utils::unwrap_or_return};

impl MeshGraph {
    pub fn halfedges_map(&mut self, predicate: impl Fn(f32) -> bool) -> HashMap<HalfedgeId, f32> {
        let mut halfedges_map = HashMap::new();

        for (he_id, he) in &self.halfedges {
            let twin_id = unwrap_or_return!(he.twin, "Twin missing", halfedges_map);

            let id = he_id.min(twin_id);

            if halfedges_map.contains_key(&id) {
                continue;
            }
            let len_sqr = he.length_squared(self);

            if predicate(len_sqr) {
                halfedges_map.insert(id, len_sqr);
            }
        }

        halfedges_map
    }
}
