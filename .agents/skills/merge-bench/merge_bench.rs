//! Per-merge cost of `merge_vertices_one_rings` on closed icospheres of growing size.
//!
//! Copied into `examples/` and run by `run.sh`; see `SKILL.md`.
//!
//! Env vars: `SUB` (one subdivision level, default all of 3..=7), `N` (merges per
//! mesh, default 10), `REPS` (fresh meshes per level, default 3; the median is
//! reported).

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

/// Mean time per merge over `n` merges of roughly antipodal vertex pairs. The
/// vertex search runs outside the timed region.
fn run(subdivisions: u8, n: usize) -> (usize, Duration) {
    let mut mesh: MeshGraph = IcoSphere {
        radius: 1.0,
        subdivisions,
    }
    .into();
    let halfedges = mesh.halfedges.len();
    let mut total = Duration::ZERO;
    for i in 0..n {
        // Fibonacci directions so the merges land on separate patches.
        let y = 1.0 - (i as f32 + 0.5) / n as f32 * 2.0;
        let r = (1.0 - y * y).sqrt();
        let phi = i as f32 * 2.399_963;
        let dir = Vec3::new(r * phi.cos(), y, r * phi.sin());
        let v1 = closest(&mesh, dir);
        let v2 = closest(&mesh, -dir * 0.99);
        let mut marked_halfedges = HashSet::default();
        let mut marked_vertices = HashSet::default();
        let start = Instant::now();
        mesh.merge_vertices_one_rings(v1, v2, 0.0001, &mut marked_halfedges, &mut marked_vertices);
        total += start.elapsed();
    }
    (halfedges, total / n as u32)
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

    println!("{n} merges per mesh, median of {reps} meshes");
    for subdivisions in levels {
        let mut per_merge = Vec::with_capacity(reps);
        let mut halfedges = 0;
        for _ in 0..reps {
            let (h, t) = run(subdivisions, n);
            halfedges = h;
            per_merge.push(t);
        }
        per_merge.sort();
        let ms = |d: Duration| d.as_secs_f64() * 1000.0;
        println!(
            "subdiv {subdivisions}: {halfedges:>8} halfedges, {:>7.3} ms per merge (min {:.3}, max {:.3})",
            ms(per_merge[reps / 2]),
            ms(per_merge[0]),
            ms(per_merge[reps - 1]),
        );
    }
}
