# Gravity Particle Simulation (KDTree Usage Example)

> **Disclaimer:** This entire usage example (Simulation.h/.cpp, main.cpp) was written with the help of
> Claude Sonnet 5 (Anthropic's AI coding assistant). This project is not meant to be used in production but
> merely to be regarded as a proof of concept and an usage example.

An N-body gravity simulation that uses this repository's `KDTree` to speed up the
pairwise force calculation. Every step, the current particle positions are used to
build (or rebuild) a `KDTree`. Each leaf node's particles are then collapsed into a
single cluster: its combined mass, located at its center of mass. Forces are then
computed per particle as:

- **direct, pairwise** for every other particle sharing the same leaf node (they're
  spatially close, so no approximation is used), and
- **approximated** for every other leaf/cluster, using that cluster's combined mass
  and center of mass as a single point-mass source.

This turns an O(n²) force calculation into roughly O(n · (leaf size + leaf count)),
which is cheaper whenever particles are reasonably spread out. All particles move
under gravity every step, so by default the tree is kept in sync via
`KDTree::rebuildTreeIfNeeded()`, which only performs a full rebuild once a particle
has actually moved outside its leaf node's bounding box, rather than unconditionally
rebuilding every step (`--no-adaptive-rebuild` forces an unconditional
`KDTree::rebuildTree()` instead, for comparison). Positions are handed to the
`KDTree` with `copyVertices = false` so it can be rebuilt in place from the
simulation's own position buffer without re-supplying every vertex.

Integration uses kick-drift-kick leapfrog, a standard symplectic method for
gravitational N-body systems, with Plummer softening to avoid singular forces
between particles that pass very close to each other.

### Comparing against brute force

Pass `--brute-force` to disable the `KDTree` entirely: it is never built or queried,
and every particle's acceleration is computed via a direct O(n²) pairwise sum instead.
This is the same physics (same softening, same gravitational constant) through a
different code path, so running the same particle count/seed with and without
`--brute-force` and comparing the `elapsed_ms` column is a direct measurement of what
the KDTree leaf-clustering approximation buys (or costs) you at that particle count —
brute force wins at very low particle counts where tree-building overhead dominates,
and loses increasingly badly as particle count grows.

### Measuring the accuracy cost

Speed isn't free: collapsing a leaf's particles into one point mass is an approximation,
so it introduces a small force error at every step. Pass `--compare-accuracy` to measure
it directly — instead of a normal run, this advances a KDTree-approximated simulation and
a brute-force one side by side from identical initial conditions (same seed, so both
start from byte-identical positions/velocities/masses) and reports, per step, how far
their particle positions and kinetic energy have drifted apart
(`step,rms_position_error,max_position_error,kinetic_energy_kdtree,kinetic_energy_brute_force,relative_energy_error`).

Because N-body gravity is chaotic, this divergence isn't just the raw per-step force
error — it compounds, so expect it to start near zero and grow over the course of a run
even though the same small approximation is being made every single step. `--compare-accuracy`
ignores `--kd-tree`/`--brute-force` (it always runs both) and doesn't write CSV frames.

## Building

This is a standalone CMake project with its own `CMakeLists.txt`; it pulls in the
KDTree library from the parent repository via `add_subdirectory(..)`, so the two can
be configured/built independently of the main project's build tree. Command-line
parsing uses [CLI11](https://github.com/CLIUtils/CLI11), fetched the same way the
main project's own executable fetches it (see `cmake/cli11.cmake`).

```bash
cmake -S usage-examples -B usage-examples/build
cmake --build usage-examples/build
```

The first configure will fetch the KDTree library's own dependencies (spdlog,
Thrust, etc.) exactly as it would for a normal build of this repository — see the
top-level [README](../README.md) for details.

## Running

```bash
./usage-examples/build/gravity_sim --particles 300 --steps 200
```

Run `--help` for the full list of options (particle count, time step, softening,
gravitational constant, initial spin, KDTree plane selection algorithm, output
directory/interval, ...).

Each run prints one CSV-style line per step to stdout
(`step,elapsed_ms,clusters,rebuilt,kinetic_energy`, where `rebuilt` is `rebuild` if
that step actually rebuilt the KDTree and `noop` if `rebuildTreeIfNeeded()` decided it
could reuse the existing tree — always `noop` in `--brute-force` mode, since the tree
is never touched) and, by default, writes a `frame_NNNNNN.csv` snapshot of every
particle's position/velocity/mass every 5 steps into `output/` (configurable via
`--output-dir` / `--output-interval`), for later plotting or inspection.
