#!/usr/bin/env bash
# Runs merge_bench.rs as a temporary cargo example in the current checkout and
# removes it again afterwards. Env vars (SUB, N, REPS) pass through to the bench.
# Extra arguments go to `cargo run` (e.g. `--features instrumentation`).
set -euo pipefail

skill_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(git -C "$skill_dir" rev-parse --show-toplevel)"
example="$repo_root/examples/merge_bench.rs"

if [[ -e "$example" ]]; then
    echo "$example already exists; remove it or run the bench from it directly" >&2
    exit 1
fi

# The Xcode linker on the maintainer's Mac can't link this crate; use the
# Command Line Tools one when it is there and nothing else was configured.
clt=/Library/Developer/CommandLineTools
if [[ "$(uname)" == Darwin && -z "${RUSTFLAGS:-}" && -x "$clt/usr/bin/clang" ]]; then
    export RUSTFLAGS="-C linker=$clt/usr/bin/clang -C link-arg=-isysroot -C link-arg=$clt/SDKs/MacOSX.sdk"
fi

created_dir=0
[[ -d "$repo_root/examples" ]] || created_dir=1
mkdir -p "$repo_root/examples"
cleanup() {
    rm -f "$example"
    if [[ $created_dir == 1 ]]; then rmdir "$repo_root/examples" 2>/dev/null || true; fi
}
trap cleanup EXIT

cp "$skill_dir/merge_bench.rs" "$example"
cd "$repo_root"
cargo run --quiet --release --example merge_bench "$@"
