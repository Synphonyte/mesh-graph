---
name: merge-bench
description: Benchmark the per-merge cost of `merge_vertices_one_rings` on closed icospheres of growing size (3.8k–983k halfedges) and compare it with recorded reference numbers. Use after changing anything on the merge path (merge_one_ring, remove_face, rim scans, collapse/flip helpers it calls), when asked whether a change made merging slower, or to check that per-merge cost doesn't grow with mesh size.
---

# Merge benchmark

freestyle-sculpt calls `merge_vertices_one_rings` hundreds of times per stroke, so
its cost has to stay independent of mesh size. This benchmark merges 10 roughly
antipodal vertex pairs on `IcoSphere` subdivisions 3..=7 and reports the mean time
per merge, as the median over 3 fresh meshes. Only the merge call is timed; finding
the vertex pairs is not.

## Run

From anywhere in the checkout:

```bash
.claude/skills/merge-bench/run.sh
```

`run.sh` copies `merge_bench.rs` into `examples/`, runs it with `--release`, and
deletes it again (along with `examples/` if it created it). It sets the macOS linker
`RUSTFLAGS` itself when `RUSTFLAGS` is unset. Knobs (env vars): `SUB=7` (one level
only), `N=30` (merges per mesh), `REPS=5` (meshes per level). Extra arguments go to
`cargo run`, e.g. `run.sh --features instrumentation`. Don't compare those numbers
with plain builds.

A full run takes about a minute after the build. Most of that is the untimed
vertex search on the 983k mesh.

## Reading the result

What matters is how the numbers **grow with the subdivision level**. Each level has
4x the halfedges of the one before. A merge that is local should cost about the same
at every level. If the time grows about 4x per level, something on the merge path
touches the whole mesh. Absolute numbers depend on the machine. Subdivisions 3 and 4
are noisy (look at min/max). Rely on levels 5–7.

## Reference numbers

Measured with this script on the maintainer's Mac (Apple Silicon, release build),
ms per merge:

| commit | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|
| `e0b3091` (before local `remove_face`) | 0.13 | 0.36 | 0.83 | 3.2 | 15.7 |
| `134945e` (`remove_face` finds members locally) | 0.11 | 0.18 | 0.23 | 0.78 | 3.0 |

At `134945e` the remaining growth (about 4x per level from 5 to 7) is not in
`remove_face`, which costs 6–15 µs per merge at every level. It matches the
whole-mesh boundary scans added after v0.11.0, e.g. the `outgoing_halfedges` sweep
in `src/ops/merge_one_ring/rim_probe.rs`. Once those are local, levels 5–7 should
come out roughly flat. Add a row for every change that moves these numbers.

## Comparing against another commit

Numbers from different machines or days aren't comparable, so measure the old commit
next to the new one:

```bash
git worktree add --detach /tmp/merge-bench-base <commit>
mkdir -p /tmp/merge-bench-base/.claude/skills
cp -r .claude/skills/merge-bench /tmp/merge-bench-base/.claude/skills/
CARGO_TARGET_DIR="$PWD/target" /tmp/merge-bench-base/.claude/skills/merge-bench/run.sh
git worktree remove --force /tmp/merge-bench-base
```

Copy the skill in as shown, because older commits don't have it. Sharing
`CARGO_TARGET_DIR` reuses the compiled dependencies. Use your scratchpad instead of
`/tmp` when you have one.

## Finding what grows

The sandbox may block `sample` and other profilers. Instead, time the suspect
function. Wrap it in a temporary `pub static` `AtomicU64` that adds `elapsed()`
nanoseconds, re-export it from `lib.rs`, then print and reset it per level in
`merge_bench.rs`. A suspect whose share stays flat across levels is not the cause.
Revert the timer before committing. To find candidates, grep the merge path for
whole-mesh iteration: `halfedges.iter()`, `.values()`, `.keys()`,
`outgoing_halfedges.iter()`.
