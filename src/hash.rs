//! The crate's hash maps and sets.
//!
//! Both aliases use a fixed-seed [`foldhash`] builder, so iteration order is a
//! pure function of the insertion sequence: the same input produces the same
//! ids, allocation order and fingerprints on every run.
//!
//! A randomly seeded map cannot be introduced by accident: `clippy.toml`
//! disallows naming `hashbrown`'s and `std`'s map and set types (and the
//! random builders) anywhere but here, and CI runs clippy with `-D warnings`.
//! Turning off `hashbrown`'s `default-hasher` feature in `Cargo.toml` is not
//! enough on its own - Cargo unifies features across the build, and parry3d
//! enables it.
//!
//! The order is fixed, but it still follows the hashes of the keys, not the
//! mesh: an algorithm that picks or drains elements through a set depends on
//! slotmap key versions it has no business seeing. The order-independence
//! tests vary those versions (`utils::mesh_with_key_generation`) to catch that.

/// A [`hashbrown::HashMap`] with the crate's fixed-seed builder.
#[allow(clippy::disallowed_types)]
pub type HashMap<K, V> = ::hashbrown::HashMap<K, V, foldhash::fast::FixedState>;

/// A [`hashbrown::HashSet`] with the crate's fixed-seed builder.
#[allow(clippy::disallowed_types)]
pub type HashSet<T> = ::hashbrown::HashSet<T, foldhash::fast::FixedState>;
