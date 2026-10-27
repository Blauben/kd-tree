#!/bin/bash
#SBATCH -o /dss/dsshome1/0C/ge47pos2/kd-tree/kd-tree-bench.%j.out
#SBATCH -e /dss/dsshome1/0C/ge47pos2/kd-tree/kd-tree-bench.%j.err
#SBATCH -D /dss/dsshome1/0C/ge47pos2/kd-tree
#SBATCH -J Blauben-kd-tree-bench
#SBATCH --get-user-env
#SBATCH --clusters=cm4
#SBATCH --partition=cm4_std
#SBATCH --nodes=2
#SBATCH --ntasks-per-node=112
#SBATCH --mail-type=ALL
#SBATCH --mail-user=ben.frauenknecht@tum.de
#SBATCH --export=NONE
#SBATCH --qos=cm4_std
#SBATCH --time=08:00:00

# Usage:
#   sbatch script/slurm_benchmark.sh   # compute node: cmake configure + build, then run all missing chunks
#                                      # rerun after a timeout to continue with the missing chunks
#   script/slurm_benchmark.sh status   # show which chunks are done
#   script/slurm_benchmark.sh merge    # merge chunks into one json per build

set -euo pipefail

# sbatch runs a copy of this script, so rely on the submit dir (-D) instead of $0
KDTREE_DIR="${KDTREE_DIR:-$(pwd)}"
BUILD_ROOT="${BUILD_ROOT:-$KDTREE_DIR/build}"
RESULTS_DIR="${RESULTS_DIR:-$KDTREE_DIR/benchmark-results}"

# benchmark settings
REPETITIONS="${REPETITIONS:-5}"
FILTER="${FILTER:-/BuildTree}"
BUILDS=(cpp omp)
# file names the report notebook picks up (figures/data/benchmarks/after-*.json)
OUT_NAME_cpp="${OUT_NAME_cpp:-after-optimization-cpp.json}"
OUT_NAME_omp="${OUT_NAME_omp:-after-optimization-omp.json}"

# module names differ between cluster updates, check with `module avail gcc cmake python`
MODULES="${MODULES:-cmake gcc/15.2.0}"

log() { echo "[$(date '+%F %T')] $*"; }

load_modules() {
    # non-interactive shells don't always have the module function, so pull it in
    if ! command -v module >/dev/null 2>&1; then
        for f in /etc/profile.d/lmod.sh /etc/profile.d/modules.sh; do
            # shellcheck disable=SC1090
            [[ -f "$f" ]] && source "$f" && break
        done
    fi
    if command -v module >/dev/null 2>&1; then
        # lmod scripts reference unset variables
        set +u
        # shellcheck disable=SC2086
        module load $MODULES
        set -u
    else
        echo "warning: no module command, using whatever is on PATH" >&2
    fi
    export CC="${CC:-gcc}" CXX="${CXX:-g++}"
    # python module ships an older libstdc++ that shadows gcc's (CXXABI not found), gcc libs first
    local libstdcxx
    libstdcxx="$("$CXX" -print-file-name=libstdc++.so.6)"
    if [[ "$libstdcxx" == /* ]]; then
        export LD_LIBRARY_PATH="$(dirname "$libstdcxx")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
}

build_dir() { echo "$BUILD_ROOT/benchmark-$1"; }

setup() {
    load_modules
    cd "$KDTREE_DIR"
    for b in "${BUILDS[@]}"; do
        log "building $b"
        cmake -S "$KDTREE_DIR" -B "$(build_dir "$b")" \
            -DCMAKE_BUILD_TYPE=Release \
            -DKD_TREE_PARALLELIZATION="${b^^}" \
            -DKD_TREE_LOGGING_LEVEL=WARN \
            -DBUILD_KD_TREE_TIME_MEASUREMENT=ON \
            -DBUILD_KD_TREE_TESTS=OFF \
            -DBUILD_KD_TREE_PYTHON_INTERFACE=OFF \
            -DBUILD_KD_TREE_DOCS=OFF
        cmake --build "$(build_dir "$b")" --target KDTree_time -j "${SLURM_CPUS_ON_NODE:-$(nproc)}"
    done
}

# list "<mesh>/<algorithm>" pairs matching FILTER
list_chunks() {
    "$(build_dir "$1")/KDTree_time" --benchmark_list_tests --benchmark_filter="$FILTER" \
        | awk -F/ '{print $1"/"$2}' | sort -u
}

chunk_file() { echo "$RESULTS_DIR/$1/${2//\//__}.json"; }

# runs the missing chunks, finished ones are skipped
run() {
    export OMP_NUM_THREADS="${OMP_NUM_THREADS:-${SLURM_CPUS_ON_NODE:-$(nproc)}}" OMP_PROC_BIND=close OMP_PLACES=cores
    log "host $(hostname), OMP_NUM_THREADS=$OMP_NUM_THREADS, kd-tree $(git -C "$KDTREE_DIR" rev-parse --short HEAD)"
    for b in "${BUILDS[@]}"; do
        [[ -x "$(build_dir "$b")/KDTree_time" ]] || { echo "error: $b build is missing" >&2; exit 1; }
        mkdir -p "$RESULTS_DIR/$b"
        # binary loads meshes from ./resources, so run it from the build dir
        pushd "$(build_dir "$b")" >/dev/null
        # a failing binary inside < <(...) is silently ignored, so grab the list first
        local chunks
        chunks="$(list_chunks "$b")"
        [[ -n "$chunks" ]] || { echo "error: $b lists no benchmarks, binary broken?" >&2; exit 1; }
        while read -r chunk; do
            local out
            out="$(chunk_file "$b" "$chunk")"
            [[ -s "$out" ]] && continue
            log "$b: $chunk"
            # write to .part first so a job killed mid-chunk doesn't leave a broken file around
            ./KDTree_time \
                --benchmark_filter="^${chunk}/" \
                --benchmark_repetitions="$REPETITIONS" \
                --benchmark_out="$out.part" \
                --benchmark_out_format=json \
                --benchmark_format=console
            mv "$out.part" "$out"
        done <<<"$chunks"
        popd >/dev/null
    done
    log "all chunks done, now run: $0 merge"
}

status() {
    load_modules
    for b in "${BUILDS[@]}"; do
        if [[ ! -x "$(build_dir "$b")/KDTree_time" ]]; then
            echo "$b: not built yet"
            continue
        fi
        local done=0 total=0
        while read -r chunk; do
            total=$((total + 1))
            [[ -s "$(chunk_file "$b" "$chunk")" ]] && done=$((done + 1))
        done < <(list_chunks "$b")
        echo "$b: $done/$total chunks done"
    done
}

merge() {
    for b in "${BUILDS[@]}"; do
        local name="OUT_NAME_$b"
        python3 - "$RESULTS_DIR/$b" "$RESULTS_DIR/${!name}" <<'EOF'
import json, sys
from pathlib import Path

chunk_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
chunks = sorted(chunk_dir.glob("*.json"))
if not chunks:
    sys.exit(f"no chunks in {chunk_dir}")
merged = None
for path in chunks:
    data = json.loads(path.read_text())
    if merged is None:
        merged = data
    else:
        merged["benchmarks"].extend(data["benchmarks"])
out.write_text(json.dumps(merged, indent=2))
print(f"{out}: {len(merged['benchmarks'])} entries from {len(chunks)} chunks")
EOF
    done
    echo "copy $RESULTS_DIR/after-optimization-{cpp,omp}.json to IDP_Report/figures/data/benchmarks/"
}

case "${1:-}" in
    "") setup; run ;;
    setup) setup ;;
    status) status ;;
    merge) merge ;;
    *) sed -n '17,22p' "$0"; exit 1 ;;
esac
