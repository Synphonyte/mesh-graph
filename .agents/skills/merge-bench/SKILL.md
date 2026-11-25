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
.agents/skills/merge-bench/run.sh
```

`run.sh` copies `merge_bench.rs` into `examples/`, runs it with `--release`, and
deletes it again (along with `examples/` if it created it). It sets the macOS linker
`RUSTFLAGS` itself when `RUSTFLAGS` is unset. Knobs (env vars): `SUB=7` (one level
only), `N=30` (merges per mesh), `REPS=3` (meshes per level), `PAIR=ring1` (which
vertices to merge, see below). Extra arguments go to `cargo run`, e.g.
`run.sh --features instrumentation`. Don't compare those numbers with plain builds.

A full run takes about a minute after the build. Most of that is the untimed
vertex search on the 983k mesh.

## Reading the result

What matters is how the numbers **grow with the subdivision level**. Each level has
4x the halfedges of the one before. A merge that is local should cost about the same
at every level. If the time grows about 4x per level, something on the merge path
touches the whole mesh. Absolute numbers depend on the machine. Subdivisions 3 and 4
are noisy (look at min/max). Rely on levels 5–7.

Compare commits by interleaving their runs in one session, and throw away the first run
after a build: levels 3–4 read 2–3x slower on that one because the caches are cold. One
run per commit is a coin flip — `0b60451` alone reads 0.072–0.093 at level 7 between
sessions and 0.058–0.182 at level 3 across five consecutive runs on the same machine.
Background load (an editor, a browser, a chat app) is worth ±10% at every level. Three
rounds alternating the two checkouts is enough to resolve a 5% difference.

The default merge takes two antipodal vertices, so the rim it opens is two disjoint loops
and the planner caps them with an open-cap band: 12 faces per merge, whatever the level.
`PAIR` picks another distance between the two vertices, which changes the rim and with it
the plan:

| `PAIR` | rim the removals open | plan | faces |
|---|---|---|---|
| `antipodal` (default) | two disjoint loops | open-cap band | 12 |
| `ring1` | one loop of two runs | bigon zip | 5–6 |
| `ring2` | one loop, two arcs with a strip run between them | lobe zip | 9 |
| `ring3` | two disjoint loops | open-cap band | 11 |

The output ends with the faces each merge planned, so a merge that stopped planning reads
0.0 instead of hiding behind a faster time, and every level asserts that the mesh ends with
0 boundary halfedges. What none of the modes reach is the triangulation and center-fan
fallbacks: they need blocked diagonals, which a clean icosphere doesn't have. The corpus
tests cover those.

## Reference numbers

Measured with this script on the maintainer's Mac (Apple Silicon, release build),
ms per merge:

| commit | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|
| `e0b3091` (before local `remove_face`) | 0.13 | 0.36 | 0.83 | 3.2 | 15.7 |
| `134945e` (`remove_face` finds members locally) | 0.11 | 0.18 | 0.23 | 0.78 | 3.0 |
| `7ccf06a` (boundary scans scoped to the merge region, old `remove_face`) | 0.23 | 0.32 | 0.69 | 2.6 | 10.1 |
| `93eeef6` (both) | 0.15 | 0.12 | 0.08 | 0.10 | 0.09 |
| after `9d76194` (merge nets removed) | 0.05 | 0.06 | 0.06 | 0.08 | 0.07 |
| `0b60451` (rim-first band planner) | 0.052 | 0.050 | 0.060 | 0.064 | 0.081 |
| `42e138c` (#51 + #52 merged) | 0.060 | 0.061 | 0.067 | 0.077 | 0.084 |

The `7ccf06a` and `93eeef6` rows were measured side by side with a re-run of `134945e`, which
matched its row (0.22 / 0.79 / 3.2 at levels 5–7). At `93eeef6` the
merge is local: levels 5–7 are flat. Each of the two fixes alone still grows about
4x per level, because the other one is a whole-mesh pass: `remove_face`'s member scan
at `7ccf06a`, and the merge's five boundary scans (entry snapshot, rim snapshot, two
rim deltas, the closer's opened-edge search) at `134945e`. Add a row for every
change that moves these numbers.

The nets-removed row was measured side by side with `9d76194` at `N=30` (0.11 / 0.09 /
0.08 / 0.08 / 0.10 in the same run): dropping the closer and the parallel-edge weld makes
a merge about 20% cheaper and keeps it flat.

`0b60451` was measured at `N=30` next to `926f3d4` (v0.11.0) in one session. v0.11.0 still
grows about 4x per level - 0.116 / 0.218 / 0.650 / 2.582 / 9.630 - so the merge became
local in the unreleased work before the rim planner, not in it: levels 5-7 are within
noise of the `9d76194` row. At 983k halfedges a merge costs about 120x less than v0.11.0.
Benchmarking v0.11.0 needs the example's import changed to `hashbrown::HashSet`, since the
crate's `hash` module came later; nothing else differs.

The `42e138c` row was measured on this Mac with the two commits interleaved: five rounds
alternating `0b60451` and this one, the first round dropped for the cold caches, median per
level. Run that way, `0b60451` is 0.058 / 0.060 / 0.065 / 0.069 / 0.079, so `42e138c` costs
1.03 / 1.02 / 1.03 / 1.12 / 1.06 of it: levels 3–5 level to within 3%, levels 6–7 up 5–12%.
Repeat the same commit in another session and it moves by 20–30%, which is the machine,
not the code.

The seed maintenance in `#51` and `#52` accounts for that gap: timing
`make_outgoing_halfedge_boundary_if_possible` inside the merge gives ~108 calls and ~3 us
per merge at every level — 4.8% of the merge at level 3, 3.6% at level 7. It is O(vertices
the op touches), not O(mesh), and the common case is a single lookup because a vertex that
already has a boundary seed returns early. That is why the curve stayed flat.

Older rows above were single sessions and have not been re-measured this way. Against them
this machine reads `0b60451` at +12% / +20% / +8% / +8% / -2%, so a few points of the gap
between any two rows here is day-to-day drift; treat them as an order of magnitude, not as
a measurement of the change between commits.

### Pair modes

Measured at `0b60451`, `N=30`, ms per merge. The `antipodal` row here comes from a later
run than the table above (0.081 at level 7 there, 0.075 here), which is the run-to-run
spread to expect: compare modes within one run, not across days.

| `PAIR` | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|
| `antipodal` | 0.073 | 0.058 | 0.050 | 0.061 | 0.075 |
| `ring1` | 0.046 | 0.043 | 0.043 | 0.044 | 0.051 |
| `ring2` | 0.072 | 0.056 | 0.049 | 0.062 | 0.067 |

All three are flat, which is the point: a near pair costs no more than an antipodal one,
so the planner's cost is set by the rim, not by the mesh.

## Comparing against another commit

Numbers from different machines or days aren't comparable, so measure the old commit
next to the new one:

```bash
git worktree add --detach /tmp/merge-bench-base <commit>
mkdir -p /tmp/merge-bench-base/.agents/skills
cp -r .agents/skills/merge-bench /tmp/merge-bench-base/.agents/skills/
/tmp/merge-bench-base/.agents/skills/merge-bench/run.sh
git worktree remove --force /tmp/merge-bench-base
```

Copy the skill in as shown, because older commits don't have it. Use your
scratchpad instead of `/tmp` when you have one.

Don't point both checkouts at the same `CARGO_TARGET_DIR` to save build time. Both
build `mesh-graph` into identically named artifacts. The other checkout's library
can then end up in this checkout's binary without cargo rebuilding it, and you
silently measure the wrong code. If a run disagrees with the reference table for no
reason, `touch src/lib.rs` and run again.

## Finding what grows

The sandbox may block `sample` and other profilers. Instead, time the suspect
function. Wrap it in a temporary `pub static` `AtomicU64` that adds `elapsed()`
nanoseconds, re-export it from `lib.rs`, then print and reset it per level in
`merge_bench.rs`. A suspect whose share stays flat across levels is not the cause.
Revert the timer before committing. To find candidates, grep the merge path for
whole-mesh iteration: `halfedges.iter()`, `.values()`, `.keys()`,
`outgoing_halfedges.iter()`.
