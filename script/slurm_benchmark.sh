#!/bin/bash
#SBATCH -o /dss/dsshome1/0C/ge47pos2/kd-tree/kd-tree-bench.%j.out
#SBATCH -e /dss/dsshome1/0C/ge47pos2/kd-tree/kd-tree-bench.%j.err
#SBATCH -D /dss/dsshome1/0C/ge47pos2/kd-tree
#SBATCH -J Blauben-kd-tree-bench
#SBATCH --get-user-env
#SBATCH --clusters=cm4
#SBATCH --partition=cm4_tiny
#SBATCH --nodes=1
#SBATCH --cpus-per-task=112
#SBATCH --mail-type=ALL
#SBATCH --mail-user=ben.frauenknecht@tum.de
#SBATCH --export=NONE
#SBATCH --qos=cm4_tiny
#SBATCH --time=12:00:00

# Usage:
#   script/slurm_benchmark.sh setup    # login node: cmake configure, fetches deps so it needs internet
#   sbatch script/slurm_benchmark.sh   # compute node: run mode (default), build + run all missing chunks
#                                      # rerun after a timeout to continue with the missing chunks
#   script/slurm_benchmark.sh status   # show which chunks are done
#   script/slurm_benchmark.sh merge    # merge chunks into one json per build
#   sbatch --export=BENCH_THREADS=56 script/slurm_benchmark.sh   # other thread count, default is all allocated cpus
#   script/slurm_benchmark.sh probe    # quick check: prints cpu affinity, runs one small benchmark with 1..N threads
#                                      # (Time vs CPU column shows how many threads really work). PROBE_FILTER to change it

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
# SETUP_MODULES: used by `setup` only (python is needed there for the open3d scaling).
# MODULES: used by run/status. It must NOT contain python: that module puts its own (old)
# libstdc++ on LD_LIBRARY_PATH, which breaks binaries built with a newer gcc
# (CXXABI_1.3.15 not found). In MODULES gcc comes last so its libstdc++ wins; load_modules also
# prepends the compiler's libstdc++ dir to LD_LIBRARY_PATH, so the order in SETUP_MODULES is safe.
# The merge step uses the system python3.
SETUP_MODULES="${SETUP_MODULES:-cmake gcc/15.2.0 python/3.10.12-extended}"
MODULES="${MODULES:-cmake gcc/15.2.0}"

log() { echo "[$(date '+%F %T')] $*"; }

# usage: load_modules "<module list>"
load_modules() {
    module load $1
    export CC="${CC:-gcc}" CXX="${CXX:-g++}"
    # make sure the runtime libstdc++ is the one belonging to the compiler that built the binary,
    # regardless of what other modules or the environment put on LD_LIBRARY_PATH
    local libdir
    libdir="$(dirname "$(readlink -f "$("$CXX" -print-file-name=libstdc++.so)")")"
    export LD_LIBRARY_PATH="$libdir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    log "using $("$CXX" --version | head -n1), libstdc++ from $libdir"
}

build_dir() { echo "$BUILD_ROOT/benchmark-$1"; }

configure() {
    cd "$KDTREE_DIR"
    for b in "${BUILDS[@]}"; do
        log "configuring $b"
        cmake -S "$KDTREE_DIR" -B "$(build_dir "$b")" \
            -DCMAKE_BUILD_TYPE=Release \
            -DKD_TREE_PARALLELIZATION="${b^^}" \
            -DKD_TREE_LOGGING_LEVEL=WARN \
            -DBUILD_KD_TREE_TIME_MEASUREMENT=ON \
            -DBUILD_KD_TREE_TESTS=OFF \
            -DBUILD_KD_TREE_PYTHON_INTERFACE=OFF \
            -DBUILD_KD_TREE_DOCS=OFF
    done
}

build() {
    for b in "${BUILDS[@]}"; do
        # CMakeCache.txt also survives a failed configure, this one is only written on success
        [[ -f "$(build_dir "$b")/cmake_install.cmake" ]] || { echo "error: $b not configured, run setup first" >&2; exit 1; }
        log "building $b"
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
    # don't inherit OMP_NUM_THREADS, the login env (--get-user-env) sets it to 1. override via BENCH_THREADS
    # (nproc itself reads OMP_NUM_THREADS, so drop it there too)
    # no OMP_PROC_BIND/OMP_PLACES: with close/cores libgomp put every thread on place {0,80} -> 1 core total
    export OMP_NUM_THREADS="${BENCH_THREADS:-${SLURM_CPUS_ON_NODE:-$(env -u OMP_NUM_THREADS nproc)}}"
    unset OMP_PROC_BIND OMP_PLACES
    # show which cpus this job is allowed to run on, useful for debugging
    grep Cpus_allowed_list /proc/self/status
    # git is not on PATH on the compute nodes (--export=NONE), so the revision is optional
    local rev=unknown
    if command -v git >/dev/null 2>&1; then
        rev="$(git -C "$KDTREE_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    fi
    log "host $(hostname), OMP_NUM_THREADS=$OMP_NUM_THREADS, kd-tree $rev"
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

# quick sanity check of the cpus we actually get, without writing any results
probe() {
    # plain nproc returns OMP_NUM_THREADS if set (=1 from the login env), so drop it for the affinity count
    local cpus max t
    cpus="$(env -u OMP_NUM_THREADS -u OMP_THREAD_LIMIT nproc)"
    log "host $(hostname), affinity cpus=$cpus, SLURM_CPUS_ON_NODE=${SLURM_CPUS_ON_NODE:-unset}, SLURM_JOB_ID=${SLURM_JOB_ID:-none}"
    grep -E 'Cpus_allowed_list' /proc/self/status
    # a cgroup cpu quota caps cpu time without showing up in the affinity mask ("max" = no quota).
    # can sit on any parent cgroup, so walk up the whole chain
    local cg
    cg="$(sed -n 's/^0:://p' /proc/self/cgroup)"
    while :; do
        echo "cgroup ${cg:-/} cpu.max: $(cat "/sys/fs/cgroup$cg/cpu.max" 2>/dev/null || echo n/a)"
        [[ -z "$cg" || "$cg" == / ]] && break
        cg="${cg%/*}"
    done
    # PROBE_MAX_THREADS to stay polite on the login node
    max="${PROBE_MAX_THREADS:-$cpus}"
    # code-independent check: $max busy loops for 2s, cpu/wall should be ~$max without a quota
    local TIMEFORMAT="busy loops x$max: wall %R s, user %U s (user/wall = cpus actually granted)"
    time {
        for ((t = 0; t < max; t++)); do timeout 2 bash -c 'while :; do :; done' & done
        wait
    } || true
    pushd "$(build_dir omp)" >/dev/null
    local filter="${PROBE_FILTER:-^ShapeSFMMesh/BuildTree(Squared|LogSquared)/0/}"
    # what libgomp makes of the pinning: place list and where each thread ends up
    log "pinning as the benchmark run does it (close/cores), $max threads:"
    OMP_NUM_THREADS=$max OMP_PROC_BIND=close OMP_PLACES=cores OMP_DISPLAY_ENV=true OMP_DISPLAY_AFFINITY=true \
        ./KDTree_time --benchmark_filter="^ShapeSFMMesh/BuildTreeLogSquared/0/" --benchmark_min_time=0.01s 2>&1 \
        | grep -E "OMP_PLACES|OMP_PROC_BIND|affinity" | sort -u | head -n $((max + 4)) || true
    # powers of two up to nproc, plus nproc itself. once pinned like the benchmark run, once unpinned
    local bind
    for bind in "OMP_PROC_BIND=close OMP_PLACES=cores" "OMP_PROC_BIND=false"; do
        log "--- $bind"
        for t in $(t=1; while ((t < max)); do echo $t; t=$((t * 2)); done; echo "$max"); do
            echo "OMP_NUM_THREADS=$t"
            env $bind OMP_NUM_THREADS=$t ./KDTree_time --benchmark_filter="$filter" \
                --benchmark_min_time=0.2s --benchmark_format=console 2>&1 | grep -E "^[A-Za-z0-9]+/" || true
        done
    done
    popd >/dev/null
}

case "${1:-}" in
    ""|run) load_modules "$MODULES"; build; run ;;
    setup) load_modules "$SETUP_MODULES"; configure ;;
    status) load_modules "$MODULES"; status ;;
    merge) merge ;;
    probe) load_modules "$MODULES"; BUILDS=(omp); build; probe ;;
    *) sed -n '17,25p' "$0"; exit 1 ;;
esac
