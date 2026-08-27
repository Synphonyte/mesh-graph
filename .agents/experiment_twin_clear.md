# Twin-clear local-vs-baseline experiment (dangling-twin hunt, session 5)

## Goal

The committed `clear_twins_to` (O(H) full scan over all halfedges, nulling every
surviving `twin` that references a removed id) is correct but expensive. Replace it
with an O(|removed_ids|) local clear and prove equivalence experimentally, then
remove the O(H) baseline.

## Local solution (final, in `src/lib.rs`)

For every removed id `r`, find `r.twin` (the partner). If the partner survives and
isn't removed in the same batch, and its `.twin` provably points back at `r`, null
it. This is exactly what the O(H) scan does **iff twin symmetry holds at removal
time** (the only halfedge that can reference `r` is its partner, because all twin
writes are symmetric pair writes).

The experiment instrumented every `clear_twins_to` call to replay the O(H) baseline
afterwards (`MESH_GRAPH_VERIFY_TWIN_CLEAR=1`) and to scan settled op boundaries for
new one-sided pairs (`MESH_GRAPH_ONE_SIDED_SCAN=1`). All instrumentation was
removed again; only the Layer-4 probe (`MESH_GRAPH_DANGLING_CHECK=1`) remains.

## Findings

1. **First local-only runs (b1/b2): VERIFY-FAIL fired** — surviving halfedges kept
   a `twin` pointing at removed ids, and one run panicked downstream
   (`collapse.rs:123` stale SlotMap key). The local clear alone is *not* equivalent:
   the mesh contained **one-sided twin chains** (`u.twin = t` while `t.twin != u`),
   so the referrer of a removed halfedge is not always its partner.

2. **Creator identified**: `weld_faces_at` (used by `split_regions_at_edge` /
   `split_regions_at_vertex` inside `make_vertex_neighborhood_manifold`) re-pairs
   `he1 ↔ he2` but *never severs the old twins' back-references* — its own doc
   comment said "the reverse twin pointers of the previous twins are not changed".
   Chains like `31151 → 30685 → 11676` were created there, then bit at the next
   removal of `30685`/`11676` (VERIFY-FAIL + `merge_one_ring` panic, b3_r1).

3. **Fix**: `weld_faces_at` now re-pairs the old partners **with each other**
   (`t1 ↔ t2` partner swap) instead of severing them — that keeps every halfedge
   paired and symmetric (the same cleanup was applied to the
   `remove_neighboring_flaps` re-pair, which also checks both livenesses before
   writing either side). An earlier severity-only variant (t1.twin = None) was
   rejected after the twinless-invariant check (below) showed it stranding live
   face members without a twin.

4. **`remove_face` hardening**: a face member with no twin (or a dead twin
   reference) used to abort the whole face removal (`unwrap_or_return` → the face
   survived with a broken member). It now removes the member with the face. Also
   reordered: `clear_twins_to` now runs *before* the removal loop (the local clear
   needs the removed halfedges to still be present to find their partners).

5. **Twinless-invariant check** ("every surviving halfedge has a twin after an
   op"): an env-gated scan at settled op boundaries + a `Drop` probe showed the
   severing variant left 4–17 twinless halfedges at the end of every run
   (thousands of settled-point sightings at `remove_face`/`collapse_edge_inner`/
   `subdivide_edge`). With the partner-swap weld: **0 twinless halfedges in 7/8
   instrumented runs**; the single residual halfedge in the remaining run traced
   to a face chain crossing into a foreign halfedge — the pre-existing Layer-4
   face-stealing bug, not twin bookkeeping.

## Results (all `log_002`, release, features `gltf,serde`, seeds random per process)

| config | runs | VERIFY-FAIL | one-sided pairs | failures |
|---|---|---|---|---|
| O(H) baseline `83265a3` | 39 | n/a | n/a | 2 (merge_one_ring, pre-existing class) |
| local clear, pre-fix | 32 | 14 total | ~50 created | 4 (collapse.rs:123, merge_one_ring, aabb) |
| local clear + sever-weld + remove_face fix (instrumented) | 32 | **0** | **0** | 5 (parry aabb ×4, selection.rs — pre-existing Layer-4 classes); twinless at every settled op + 4–17/run at drop |
| local clear + partner-swap weld (instrumented) | 8 | 0 | 0 | 0; twinless 0/0 at settled ops in 7/8 runs, 1 Layer-4-traced he in the 8th |
| same, instrumentation stripped | 16 | n/a | n/a | **0** (16/16) |

After the fixes the local clear was exactly equivalent to the O(H) baseline on every
instrumented run (0 VERIFY-FAIL, 0 new one-sided pairs), and the twinless invariant
holds at settled boundaries except for Layer-4-chain-crossing fallout.

## Files

- `src/lib.rs` — local `clear_twins_to` (Layer-4 probe kept)
- `src/ops/remove.rs` — `remove_face`: twinless-member handling + clear-before-remove order
- `src/ops/cleanup/vertex_neighborhood.rs` — weld/flap re-pairs sever old partners
- `run_twin_clear_experiment.sh` + `twin_clear_runs/` — harness and raw outputs