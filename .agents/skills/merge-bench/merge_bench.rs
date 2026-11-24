//! Per-merge cost of `merge_vertices_one_rings` on closed icospheres of growing size.
//!
//! Copied into `examples/` and run by `run.sh`; see `SKILL.md`.
//!
//! Env vars: `SUB` (one subdivision level, default all of 3..=7), `N` (merges per
//! mesh, default 10), `REPS` (fresh meshes per level, default 3; the median is
//! reported), `PAIR` (which vertices to merge, see [`Pair`]; default `antipodal`).

use std::collections::HashMap;
use std::time::{Duration, Instant};

use glam::Vec3;
use mesh_graph::{MeshGraph, VertexId, hash::HashSet, primitives::IcoSphere};

fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

fn closest(mesh: &MeshGraph, target: Vec3) -> VertexId {
    mesh.vertices
        .keys()
        .min_by(|a, b| {
            mesh.positions[*a]
                .distance_squared(target)
                .total_cmp(&mesh.positions[*b].distance_squared(target))
        })
        .unwrap()
}

/// Which vertices each merge joins. The pair decides which rim the removals
/// open, and so which part of the planner plans it.
#[derive(Clone, Copy, Debug)]
enum Pair {
    /// Roughly antipodal vertices: two disjoint loops, capped by an open-cap
    /// band. The long-standing default, so the reference table stays comparable.
    Antipodal,
    /// Vertices that many edges apart: `1` opens one loop of two runs (a bigon),
    /// `2` one loop with a strip run between two arcs, `3` two disjoint loops
    /// again.
    Ring(usize),
}

fn pair_mode() -> Pair {
    match std::env::var("PAIR").as_deref() {
        Err(_) | Ok("antipodal") => Pair::Antipodal,
        Ok("ring1") => Pair::Ring(1),
        Ok("ring2") => Pair::Ring(2),
        Ok("ring3") => Pair::Ring(3),
        Ok(other) => panic!("PAIR must be antipodal, ring1, ring2 or ring3, not {other:?}"),
    }
}

fn neighbours(mesh: &MeshGraph, vertex: VertexId) -> impl Iterator<Item = VertexId> + '_ {
    mesh.outgoing_halfedges
        .get(vertex)
        .into_iter()
        .flatten()
        .map(|he_id| mesh.halfedges[*he_id].end_vertex)
}

/// The vertex exactly `steps` edges from `start` that is farthest from it, ties
/// going to the first in id order so the choice stays deterministic.
fn farthest_at_ring(mesh: &MeshGraph, start: VertexId, steps: usize) -> VertexId {
    let mut distance = HashMap::from([(start, 0usize)]);
    let mut frontier = vec![start];
    for step in 1..=steps {
        let mut next = Vec::new();
        for vertex in frontier {
            for neighbour in neighbours(mesh, vertex) {
                if distance.contains_key(&neighbour) {
                    continue;
                }
                distance.insert(neighbour, step);
                next.push(neighbour);
            }
        }
        frontier = next;
    }

    let target = mesh.positions[start];
    let mut best = None;
    for vertex in mesh.vertices.keys() {
        if distance.get(&vertex) != Some(&steps) {
            continue;
        }
        let dist_sqr = mesh.positions[vertex].distance_squared(target);
        if best.is_none_or(|(_, best_dist_sqr)| dist_sqr > best_dist_sqr) {
            best = Some((vertex, dist_sqr));
        }
    }
    best.expect("a vertex that far from the first").0
}

/// One level's measurement.
struct Measuring {
    halfedges: usize,
    per_merge: Duration,
    faces_per_merge: f64,
}

/// Mean time per merge over `n` merges. The vertex search runs outside the timed
/// region.
fn run(subdivisions: u8, n: usize, pair: Pair) -> Measuring {
    let mut mesh: MeshGraph = IcoSphere {
        radius: 1.0,
        subdivisions,
    }
    .into();
    let halfedges = mesh.halfedges.len();
    let mut total = Duration::ZERO;
    let mut added_faces = 0usize;
    for i in 0..n {
        // Fibonacci directions so the merges land on separate patches.
        let y = 1.0 - (i as f32 + 0.5) / n as f32 * 2.0;
        let r = (1.0 - y * y).sqrt();
        let phi = i as f32 * 2.399_963;
        let dir = Vec3::new(r * phi.cos(), y, r * phi.sin());
        let v1 = closest(&mesh, dir);
        let v2 = match pair {
            Pair::Antipodal => closest(&mesh, -dir * 0.99),
            Pair::Ring(steps) => farthest_at_ring(&mesh, v1, steps),
        };
        let mut marked_halfedges = HashSet::default();
        let mut marked_vertices = HashSet::default();
        let start = Instant::now();
        let merged = mesh.merge_vertices_one_rings(
            v1,
            v2,
            0.0001,
            &mut marked_halfedges,
            &mut marked_vertices,
        );
        total += start.elapsed();
        added_faces += merged.added_faces.len();
    }

    // The premise every number here rests on: a merge closes what it opened. A
    // merge that returned early would look faster, not wrong.
    let boundary = mesh
        .halfedges
        .values()
        .filter(|he| he.face.is_none())
        .count();
    assert_eq!(
        boundary, 0,
        "subdiv {subdivisions} ({pair:?}): the mesh ends with {boundary} boundary halfedges"
    );

    Measuring {
        halfedges,
        per_merge: total / n as u32,
        faces_per_merge: added_faces as f64 / n as f64,
    }
}

fn main() {
    let levels = match std::env::var("SUB") {
        Ok(s) => {
            let s = s.parse().expect("SUB must be a subdivision level");
            s..=s
        }
        Err(_) => 3..=7u8,
    };
    let n = env_or("N", 10usize);
    let reps = env_or("REPS", 3usize).max(1);
    let pair = pair_mode();

    println!("{n} merges per mesh, {pair:?} pairs, median of {reps} meshes");
    for subdivisions in levels {
        let mut per_merge = Vec::with_capacity(reps);
        let mut halfedges = 0;
        let mut faces_per_merge = 0.0;
        for _ in 0..reps {
            let measuring = run(subdivisions, n, pair);
            halfedges = measuring.halfedges;
            faces_per_merge = measuring.faces_per_merge;
            per_merge.push(measuring.per_merge);
        }
        per_merge.sort();
        let ms = |d: Duration| d.as_secs_f64() * 1000.0;
        println!(
            "subdiv {subdivisions}: {halfedges:>8} halfedges, {:>7.3} ms per merge (min {:.3}, max {:.3}), {faces_per_merge:.1} faces",
            ms(per_merge[reps / 2]),
            ms(per_merge[0]),
            ms(per_merge[reps - 1]),
        );
    }
}
