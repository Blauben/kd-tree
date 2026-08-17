#include "Simulation.h"

#include "KDTree/util/UtilityContainer.h"

#include <cmath>
#include <random>
#include <shared_mutex>

namespace gravity_demo {

    namespace {
        using kdtree::IndexVector;
        using kdtree::Vertex;
        using namespace kdtree::util;

        std::vector<IndexVector> buildParticleShapes(const std::size_t particleCount) {
            std::vector<IndexVector> shapes{};
            shapes.reserve(particleCount);
            for (std::size_t index = 0; index < particleCount; ++index) {
                shapes.emplace_back(1, index);
            }
            return shapes;
        }

        std::vector<Vertex> generatePositions(const SimulationConfig &config, std::mt19937 &rng) {
            std::uniform_real_distribution<double> coordinate(-config.halfExtent, config.halfExtent);
            std::vector<Vertex> positions{};
            positions.reserve(config.particleCount);
            // Rejection-sample points inside the bounding sphere so the initial cloud is
            // roughly spherical rather than cube-shaped.
            while (positions.size() < config.particleCount) {
                const Vertex candidate{coordinate(rng), coordinate(rng), coordinate(rng)};
                if (dot(candidate, candidate) <= config.halfExtent * config.halfExtent) {
                    positions.push_back(candidate);
                }
            }
            return positions;
        }

        std::vector<Vertex> generateVelocities(const SimulationConfig &config, const std::vector<Vertex> &positions) {
            // Give every particle a small tangential (spin) velocity around the Z axis so
            // the cloud swirls instead of just collapsing straight inward, then remove the
            // net momentum so the whole system doesn't drift off-center over time.
            const Vertex angularVelocity{0.0, 0.0, config.initialSpin};
            std::vector<Vertex> velocities{};
            velocities.reserve(positions.size());
            for (const auto &position: positions) {
                velocities.push_back(cross(angularVelocity, position));
            }

            Vertex meanVelocity{0.0, 0.0, 0.0};
            for (const auto &velocity: velocities) {
                meanVelocity = meanVelocity + velocity;
            }
            meanVelocity = meanVelocity / static_cast<double>(velocities.size());
            for (auto &velocity: velocities) {
                velocity = velocity - meanVelocity;
            }
            return velocities;
        }

        std::vector<double> generateMasses(const SimulationConfig &config) {
            return std::vector<double>(config.particleCount, config.particleMass);
        }
    }// namespace

    Simulation::Simulation(const SimulationConfig &config)
        : _config(config),
          _positions([&config] {
              std::mt19937 rng(config.seed);
              return generatePositions(config, rng);
          }()),
          _velocities(generateVelocities(config, _positions)),
          _masses(generateMasses(config)),
          _accelerations(config.particleCount, Vertex{0.0, 0.0, 0.0}),
          // copyVertices = false: the tree stores pointers directly into _positions, so
          // mutating _positions in step() and calling rebuildTree() keeps the tree in sync
          // without having to reconstruct it (and re-hand every vertex) each step.
          _tree(_positions, buildParticleShapes(config.particleCount), config.algorithm, false) {
        _accelerations = computeNextAccelerations();
    }

    void Simulation::printKdTree() {
        std::cout << _tree;
    }

    std::vector<kdtree::Vertex> Simulation::computeNextAccelerations() {
        if (!_config.useKdTree) {
            // The KDTree is never built or queried in this mode, so there's nothing
            // meaningful to report for either of these.
            _lastClusterCount = _positions.size();
            _lastRebuildOccurred = false;
            return computeAccelerationsBruteForce();
        }
        const auto clusters = buildClusters();
        _lastClusterCount = clusters.size();
        return computeAccelerations(clusters);
    }

    std::vector<Simulation::Cluster> Simulation::buildClusters() {
        // Identifies the current root node so a rebuild can be detected below: rebuildTree()
        // (called directly, or internally by rebuildTreeIfNeeded()) always replaces it with a
        // freshly constructed one, while a skipped rebuild leaves the existing root in place.
        const auto *previousRoot = _tree.getRootNode().get();

        if (_config.adaptiveRebuild) {
            // Only performs a full rebuild if a particle has actually moved outside its leaf
            // node's bounding box since the tree was last built; otherwise the existing tree
            // (and thus its clusters) is reused as-is, which is cheaper than rebuilding
            // unconditionally every step.
            _tree.rebuildTreeIfNeeded();
        } else {
            _tree.rebuildTree();
        }
        _tree.prebuildTree();
        _lastRebuildOccurred = _tree.getRootNode().get() != previousRoot;

        std::vector<Cluster> clusters{};
        std::shared_lock lock(_tree.nodeRegister.leafNodeMutex);
        clusters.reserve(_tree.nodeRegister.leafNodes.size());
        for (const auto &weakLeaf: _tree.nodeRegister.leafNodes) {
            const auto leaf = weakLeaf.lock();
            if (leaf == nullptr) {
                continue;
            }

            Cluster cluster{};
            cluster.members = leaf->getContainedParticles();

            Vertex weightedSum{0.0, 0.0, 0.0};
            for (const auto &[particleIndex, position]: cluster.members) {
                const double mass = _masses[particleIndex];
                cluster.totalMass += mass;
                weightedSum = weightedSum + position * mass;
            }
            cluster.centerOfMass = cluster.totalMass > 0.0 ? weightedSum / cluster.totalMass : Vertex{0.0, 0.0, 0.0};
            clusters.push_back(std::move(cluster));
        }
        return clusters;
    }

    kdtree::Vertex Simulation::accelerationTowardsCluster(const Vertex &position, const Cluster &cluster) const {
        const Vertex delta = cluster.centerOfMass - position;
        const double distanceSquared = dot(delta, delta) + _config.softening * _config.softening;
        const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
        const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
        return delta * (_config.gravitationalConstant * cluster.totalMass * inverseDistanceCubed);
    }

    std::vector<kdtree::Vertex> Simulation::computeAccelerations(const std::vector<Cluster> &clusters) const {
        std::vector<Vertex> accelerations(_positions.size(), Vertex{0.0, 0.0, 0.0});

        for (const auto &cluster: clusters) {
            for (const auto &[particleIndex, particlePosition]: cluster.members) {
                Vertex acceleration{0.0, 0.0, 0.0};

                // Particles sharing a leaf node are close together, so their interactions
                // are computed directly rather than approximated.
                for (const auto &[otherIndex, otherPosition]: cluster.members) {
                    if (otherIndex == particleIndex) {
                        continue;
                    }
                    const Vertex delta = otherPosition - particlePosition;
                    const double distanceSquared = dot(delta, delta) + _config.softening * _config.softening;
                    const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
                    const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
                    acceleration = acceleration + delta * (_config.gravitationalConstant * _masses[otherIndex] * inverseDistanceCubed);
                }

                // Every other cluster/leaf is approximated as a single combined point mass
                // located at that leaf's center of mass.
                for (const auto &otherCluster: clusters) {
                    if (&otherCluster == &cluster) {
                        continue;
                    }
                    acceleration = acceleration + accelerationTowardsCluster(particlePosition, otherCluster);
                }

                accelerations[particleIndex] = acceleration;
            }
        }
        return accelerations;
    }

    std::vector<kdtree::Vertex> Simulation::computeAccelerationsBruteForce() const {
        std::vector<Vertex> accelerations(_positions.size(), Vertex{0.0, 0.0, 0.0});
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            Vertex acceleration{0.0, 0.0, 0.0};
            for (std::size_t j = 0; j < _positions.size(); ++j) {
                if (i == j) {
                    continue;
                }
                const Vertex delta = _positions[j] - _positions[i];
                const double distanceSquared = dot(delta, delta) + _config.softening * _config.softening;
                const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
                const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
                acceleration = acceleration + delta * (_config.gravitationalConstant * _masses[j] * inverseDistanceCubed);
            }
            accelerations[i] = acceleration;
        }
        return accelerations;
    }

    void Simulation::step() {
        const double halfStep = 0.5 * _config.timeStep;

        // Kick: apply half of this step's velocity change using the acceleration computed
        // at the end of the previous step (or, for the first step, at construction time).
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            _velocities[i] = _velocities[i] + _accelerations[i] * halfStep;
        }
        // Drift: move particles using the half-updated velocity.
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            _positions[i] = _positions[i] + _velocities[i] * _config.timeStep;
        }

        // Recompute accelerations from the new positions (rebuilds the KDTree once, unless
        // SimulationConfig::useKdTree is false).
        _accelerations = computeNextAccelerations();

        // Kick: apply the second half of the velocity change using the new acceleration.
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            _velocities[i] = _velocities[i] + _accelerations[i] * halfStep;
        }
    }

    double Simulation::totalKineticEnergy() const {
        double energy = 0.0;
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            energy += 0.5 * _masses[i] * dot(_velocities[i], _velocities[i]);
        }
        return energy;
    }

}// namespace gravity_demo
