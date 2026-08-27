#!/usr/bin/env bash
# Twin-clear local-vs-baseline comparison experiment (see session 5 of the
# dangling-twin hunt).
#
# Compares the O(|removed_ids|) local `clear_twins_to` against the O(H) baseline
# full scan by re-running the O(H) semantics after every local clear
# (`MESH_GRAPH_VERIFY_TWIN_CLEAR=1` in mesh-graph's clear_twins_to) and printing
#    TWIN-CLEAR VERIFY-FAIL  -> surviving halfedge still points at a removed id:
#                              local clear disagreed with the baseline (real bug)
#    TWIN-CLEAR ASYMMETRIC   -> pre-existing one-sided twin chain found while
#                              clearing; both implementations skip those identically
#                              (informational only)
#
# Usage: run_twin_clear_experiment.sh [--batch N] [--seeds N] [--parallel P]
#   --batch N   batch counter for output naming (default 0)
#   --seeds N   number of runs to execute (default 16)
#   --parallel P  concurrent runs (default 8; keep an eye on RAM)
#
# Requires: freestyle-sculpt's release lib-test binary built with
#   cargo test --release --features gltf,serde --no-run   (from freestyle-sculpt/)
# Output: FRREESTYLE_DIR ../../freestyle-sculpt if run from mesh-graph/.agents

set -u

MESH_GRAPH_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FREESTYLE_DIR="$(cd "$MESH_GRAPH_DIR/../freestyle-sculpt" && pwd)"
OUT_DIR="$MESH_GRAPH_DIR/.agents/twin_clear_runs"
mkdir -p "$OUT_DIR"

BATCH=0
SEEDS=16
PARALLEL=8

while [ $# -gt 0 ]; do
    case "$1" in
    --batch) BATCH="$2"; shift 2 ;;
    --seeds) SEEDS="$2"; shift 2 ;;
    --parallel) PARALLEL="$2"; shift 2 ;;
    *) echo "unknown arg $1"; exit 1 ;;
    esac
done

BIN="$(ls -t "$FREESTYLE_DIR"/target/release/deps/freestyle_sculpt-* | grep -v '\.d$' | grep -v '\.rlib$' | head -1)"
if [ -z "$BIN" ]; then
    echo "no freestyle_sculpt test binary found; build with: cargo test --release --features gltf,serde --no-run"
    exit 1
fi
echo "binary: $BIN"

cd "$FREESTYLE_DIR" || exit 1

run_one() {
    local i="$1"
    MESH_GRAPH_VERIFY_TWIN_CLEAR=1 "$BIN" log_002 --nocapture >"$OUT_DIR/b${BATCH}_r${i}.out" 2>&1
}
export -f run_one
export BIN OUT_DIR BATCH

seq 1 "$SEEDS" | xargs -P "$PARALLEL" -I{} bash -c 'run_one {}'

# ---- summary ---------------------------------------------------------------
ok=0
failed=0
viol=0
asym=0
for f in "$OUT_DIR"/b${BATCH}_r*.out; do
    if grep -q "test result: ok" "$f"; then ok=$((ok+1)); else failed=$((failed+1)); fi
    n=$(grep -c "TWIN-CLEAR VERIFY-FAIL" "$f" 2>/dev/null || true)
    viol=$((viol + n))
    n=$(grep -c "TWIN-CLEAR ASYMMETRIC" "$f" 2>/dev/null || true)
    asym=$((asym + n))
done

echo "================================"
echo "batch $BATCH: $((ok + failed)) runs, passed=$ok failed=$failed"
echo "VERIFY-FAIL (real mismatches): $viol (must be 0)"
echo "ASYMMETRIC (informational):    $asym"
echo "outputs in $OUT_DIR"