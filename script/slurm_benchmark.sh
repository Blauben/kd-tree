#!/usr/bin/env bash
# author: Ben Frauenknecht, ben.frauenknecht@tum.de
# Runs the BuildTree* benchmarks (sequential + OpenMP) on the LRZ Linux cluster (CoolMUC-4)
# inside an interactive job on cm4_inter.
#
# Usage (from a login node, ideally inside tmux/screen so a dropped ssh doesn't kill the job):
#   script/slurm_benchmark.sh setup    # configure both builds (login node, needs internet for pip/FetchContent)
#   script/slurm_benchmark.sh run      # salloc an interactive job, build + run all missing chunks
#   script/slurm_benchmark.sh status   # show which chunks are done
#   script/slurm_benchmark.sh merge    # merge chunks into one json per build
#
# cm4_inter caps jobs at 8h, so results are written per (mesh, algorithm) chunk. Just call `run`
# again after the job ends and it continues with whatever is still missing.
#
# Everything below can be overridden via env vars, e.g. CPUS=56 TIME=04:00:00 script/slurm_benchmark.sh run
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KDTREE_DIR="${KDTREE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
BUILD_ROOT="${BUILD_ROOT:-$KDTREE_DIR/build}"
RESULTS_DIR="${RESULTS_DIR:-$KDTREE_DIR/benchmark-results}"

# slurm settings, see https://doku.lrz.de/running-interactive-jobs-on-the-linux-cluster-1303879201.html
CLUSTER="${CLUSTER:-inter}"
PARTITION="${PARTITION:-cm4_inter}"
CPUS="${CPUS:-112}"           # whole cm4 node, nodes are shared otherwise and neighbours mess with the timings
TIME="${TIME:-04:00:00}"      # cm4_inter max
ACCOUNT="${ACCOUNT:-}"        # only if your default account has no access to the partition, see 'sacctmgr' hint in run()
QOS="${QOS:-}"
OMP_THREADS="${OMP_THREADS:-$CPUS}"

# benchmark settings
REPETITIONS="${REPETITIONS:-5}"
FILTER="${FILTER:-/BuildTree}"
BUILDS=(cpp omp)
# file names the report notebook picks up (figures/data/benchmarks/after-*.json)
OUT_NAME_cpp="${OUT_NAME_cpp:-after-optimization-cpp.json}"
OUT_NAME_omp="${OUT_NAME_omp:-after-optimization-omp.json}"

# module names differ between cluster updates, check with `module avail gcc cmake python`
MODULES="${MODULES:-cmake gcc/15.2.0}"

load_modules() {
    # non-interactive shells (srun) don't always have the module function, so pull it in
    if ! command -v module >/dev/null 2>&1; then
        for f in /etc/profile.d/lmod.sh /etc/profile.d/modules.sh; do
            # shellcheck disable=SC1090
            [[ -f "$f" ]] && source "$f" && break
        done
    fi
    if command -v module >/dev/null 2>&1; then
        # shellcheck disable=SC2086
        module load $MODULES
    else
        echo "warning: no module command, using whatever is on PATH" >&2
    fi
    # other modules (intel, mpi, ...) like to put their headers on CPATH, which can shadow glibc's
    # pthread.h and break every <mutex>/<memory> include. CLEAN_INCLUDE_ENV=1 drops those vars.
    if [[ "${CLEAN_INCLUDE_ENV:-0}" == 1 ]]; then
        unset CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH
    fi
    export CC="${CC:-gcc}" CXX="${CXX:-g++}"
    # python module ships an older libstdc++ that shadows gcc's (CXXABI not found), gcc libs first
    local libstdcxx
    libstdcxx="$("$CXX" -print-file-name=libstdc++.so.6)"
    if [[ "$libstdcxx" == /* ]]; then
        export LD_LIBRARY_PATH="$(dirname "$libstdcxx")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
}

log() { echo "[$(date '+%F %T')] $*"; }

# fail fast if the compiler can't do the basics, instead of 10min into the build
check_toolchain() {
    command -v module >/dev/null 2>&1 && module list 2>&1
    echo "CXX=$(command -v "$CXX") ($("$CXX" -dumpfullversion))"
    echo "CPATH=${CPATH:-} CPLUS_INCLUDE_PATH=${CPLUS_INCLUDE_PATH:-} C_INCLUDE_PATH=${C_INCLUDE_PATH:-}"
    local probe='#include <filesystem>
#include <mutex>
#include <memory>
int main() { std::mutex m; std::lock_guard l{m}; return std::filesystem::exists(".") ? 0 : 1; }'
    if ! echo "$probe" | "$CXX" -std=c++20 -x c++ -fsyntax-only - 2>/dev/null; then
        echo "error: $CXX can't compile <filesystem>/<mutex>, compiler output:" >&2
        echo "$probe" | "$CXX" -std=c++20 -x c++ -fsyntax-only - 2>&1 | head -20 >&2
        echo "pthread.h actually used:" >&2
        echo "$probe" | "$CXX" -std=c++20 -x c++ -fsyntax-only -H - 2>&1 | grep -m3 'pthread\.h' >&2 || true
        echo "hint: try CLEAN_INCLUDE_ENV=1, or check which module sets the include vars above" >&2
        exit 1
    fi
    echo "toolchain ok"
}

# the mesh generation step runs .venv/bin/python, so make sure it's actually there and has open3d
check_venv() {
    local py="$KDTREE_DIR/.venv/bin/python"
    if ! "$py" -c "import open3d" 2>/dev/null; then
        echo "error: $py missing or without open3d" >&2
        ls -l "$py" >&2 || true
        echo "fix: rm -rf $KDTREE_DIR/.venv and rerun '$0 setup' on the login node" >&2
        exit 1
    fi
    echo "venv ok"
}

build_dir() { echo "$BUILD_ROOT/time-$1"; }

setup() {
    load_modules
    check_toolchain
    local py
    # cmake insists on exactly the python from .python-version for the mesh-gen venv
    py="$(command -v "python$(cat "$KDTREE_DIR/.python-version")" || command -v python3)"
    for b in "${BUILDS[@]}"; do
        log "configuring $b"
        cmake -S "$KDTREE_DIR" -B "$(build_dir "$b")" \
            -DCMAKE_BUILD_TYPE=Release \
            -DKD_TREE_PARALLELIZATION="${b^^}" \
            -DKD_TREE_LOGGING_LEVEL=WARN \
            -DBUILD_KD_TREE_TIME_MEASUREMENT=ON \
            -DBUILD_KD_TREE_TESTS=OFF \
            -DBUILD_KD_TREE_PYTHON_INTERFACE=OFF \
            -DBUILD_KD_TREE_DOCS=OFF \
            -DPython3_EXECUTABLE="$py"
    done
    check_venv
    log "setup done, now run: $0 run"
}

# list "<mesh>/<algorithm>" pairs matching FILTER
list_chunks() {
    "$(build_dir "$1")/KDTree_time" --benchmark_list_tests --benchmark_filter="$FILTER" \
        | awk -F/ '{print $1"/"$2}' | sort -u
}

chunk_file() { echo "$RESULTS_DIR/$1/${2//\//__}.json"; }

# runs on the compute node
inner() {
    # never build/benchmark on the login node by accident
    [[ -n "${SLURM_JOB_ID:-}" && -n "${SLURM_STEP_ID:-}" ]] \
        || { echo "'inner' only runs inside srun, use '$0 run'" >&2; exit 1; }
    load_modules
    check_toolchain
    check_venv
    export OMP_NUM_THREADS="$OMP_THREADS" OMP_PROC_BIND=close OMP_PLACES=cores

    for b in "${BUILDS[@]}"; do
        log "building $b with $CPUS jobs"
        cmake --build "$(build_dir "$b")" --target KDTree_time -j "$CPUS"
    done

    log "host $(hostname), job $SLURM_JOB_ID, OMP_NUM_THREADS=$OMP_NUM_THREADS, kd-tree $(git -C "$KDTREE_DIR" rev-parse --short HEAD)"
    for b in "${BUILDS[@]}"; do
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

run() {
    [[ -x "$(build_dir cpp)/KDTree_time" || -f "$(build_dir cpp)/CMakeCache.txt" ]] \
        || { echo "run '$0 setup' first" >&2; exit 1; }
    mkdir -p "$RESULTS_DIR"
    local logfile
    logfile="$RESULTS_DIR/run-$(date +%Y%m%d-%H%M%S).log"
    log "requesting $CPUS cpus on $CLUSTER/$PARTITION for $TIME, log: $logfile"
    # srun doesn't inherit -c from salloc anymore, so pass it again
    local extra=()
    [[ -n "$ACCOUNT" ]] && extra+=(-A "$ACCOUNT")
    [[ -n "$QOS" ]] && extra+=(--qos "$QOS")
    # on "Invalid account or account/partition combination": list yours with
    #   sacctmgr show assoc user=$USER cluster="$CLUSTER" format=cluster,account,partition,qos%30
    # and rerun with ACCOUNT=<account> (and QOS=<qos> if one is listed)
    salloc -M "$CLUSTER" -p "$PARTITION" ${extra[@]+"${extra[@]}"} -N 1 -n 1 -c "$CPUS" -t "$TIME" --mail-user=ben.frauenknecht@tum.de --mail-type=END,FAIL \
        srun -n 1 -c "$CPUS" --cpu-bind=none \
        env CPUS="$CPUS" OMP_THREADS="$OMP_THREADS" REPETITIONS="$REPETITIONS" FILTER="$FILTER" \
            KDTREE_DIR="$KDTREE_DIR" BUILD_ROOT="$BUILD_ROOT" RESULTS_DIR="$RESULTS_DIR" MODULES="$MODULES" \
            CLEAN_INCLUDE_ENV="${CLEAN_INCLUDE_ENV:-0}" \
        bash "$SCRIPT_DIR/$(basename "$0")" inner 2>&1 | tee "$logfile"
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
    setup) setup ;;
    run) run ;;
    inner) inner ;;
    status) status ;;
    merge) merge ;;
    *) sed -n '2,13p' "$0"; exit 1 ;;
esac
