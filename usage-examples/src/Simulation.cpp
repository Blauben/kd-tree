#include "Simulation.h"

#include "KDTree/tree/LeafNode.h"
#include "KDTree/tree/SplitNode.h"
#include "KDTree/util/UtilityContainer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <shared_mutex>
#include <tuple>
#include <utility>

namespace gravity_demo {

    namespace {
        using kdtree::Vertex;
        using namespace kdtree::util;

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
          _tree(_positions, config.algorithm, false) {
        // tree is lazy and rebuildTreeIfNeeded() only asks existing leaves, so build it once up front
        // otherwise adaptive mode never builds anything and all forces are zero
        if (config.useKdTree) {
            _tree.prebuildTree();
        }
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
        if (_config.barnesHut) {
            return computeAccelerationsBarnesHut();
        }
        const auto clusters = buildClusters();
        _lastClusterCount = clusters.size();
        return computeAccelerations(clusters);
    }

    bool Simulation::updateTree() {
        if (!_config.adaptiveRebuild) {
            // Always rebuild the tree unconditionally every step, so the clusters are
            // always up to date with the current particle positions.
            _tree.rebuildTree();
            _tree.prebuildTree();
            _lastRebuildOccurred = true;
        } else if (_tree.rebuildTreeIfNeeded()) {
            // The tree was rebuilt due to a particle having moved outside its leaf's bounding box.
            _tree.prebuildTree();
            _lastRebuildOccurred = true;
        } else {
            // The tree was not rebuilt, so the clusters are still valid from the previous step.
            _lastRebuildOccurred = false;
        }
        return _lastRebuildOccurred;
    }

    std::vector<Simulation::Cluster> Simulation::buildClusters() {
        updateTree();

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
        return pairAcceleration(position, cluster.centerOfMass, cluster.totalMass);
    }

    kdtree::Vertex Simulation::pairAcceleration(const Vertex &position, const Vertex &source, const double sourceMass) const {
        const Vertex delta = source - position;
        const double distanceSquared = dot(delta, delta) + _config.softening * _config.softening;
        const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
        const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
        return delta * (_config.gravitationalConstant * sourceMass * inverseDistanceCubed);
    }

    void Simulation::flattenTree() {
        using kdtree::LeafNode;
        using kdtree::SplitNode;
        using kdtree::TreeNode;

        _barnesHutNodes.clear();
        // (node, parent index, which child slot of the parent)
        std::vector<std::tuple<std::shared_ptr<TreeNode>, int, int>> stack{{_tree.getRootNode(), -1, 0}};
        std::size_t leafCount = 0;
        while (!stack.empty()) {
            auto [node, parent, slot] = std::move(stack.back());
            stack.pop_back();
            const int index = static_cast<int>(_barnesHutNodes.size());
            _barnesHutNodes.emplace_back();
            if (parent >= 0) {
                _barnesHutNodes[parent].children[slot] = index;
            }
            if (const auto split = std::dynamic_pointer_cast<SplitNode>(node)) {
                // greater first so lesser gets popped (and numbered) first, order doesn't really matter though
                stack.emplace_back(split->getChildNode(1), index, 1);
                stack.emplace_back(split->getChildNode(0), index, 0);
            } else if (const auto leaf = std::dynamic_pointer_cast<LeafNode>(node)) {
                for (const auto &[particleIndex, position]: leaf->getContainedParticles()) {
                    _barnesHutNodes[index].members.push_back(particleIndex);
                }
                ++leafCount;
            }
        }
        _lastClusterCount = leafCount;
    }

    void Simulation::updateBarnesHutNodes() {
        constexpr double inf = std::numeric_limits<double>::infinity();
        // pre-order -> walking backwards always sees both children before their parent
        for (auto it = _barnesHutNodes.rbegin(); it != _barnesHutNodes.rend(); ++it) {
            auto &node = *it;
            node.totalMass = 0.0;
            node.boxMin = {inf, inf, inf};
            node.boxMax = {-inf, -inf, -inf};
            Vertex weightedSum{0.0, 0.0, 0.0};
            const auto include = [&node, &weightedSum](const Vertex &point, const Vertex &boxMin, const Vertex &boxMax, const double mass) {
                weightedSum = weightedSum + point * mass;
                node.totalMass += mass;
                for (int d = 0; d < 3; ++d) {
                    node.boxMin[d] = std::min(node.boxMin[d], boxMin[d]);
                    node.boxMax[d] = std::max(node.boxMax[d], boxMax[d]);
                }
            };
            if (node.children[0] < 0) {
                for (const auto particleIndex: node.members) {
                    include(_positions[particleIndex], _positions[particleIndex], _positions[particleIndex], _masses[particleIndex]);
                }
            } else {
                for (const auto childIndex: node.children) {
                    if (childIndex < 0) {
                        continue;
                    }
                    const auto &child = _barnesHutNodes[childIndex];
                    if (child.totalMass > 0.0) {
                        include(child.centerOfMass, child.boxMin, child.boxMax, child.totalMass);
                    }
                }
            }
            node.centerOfMass = node.totalMass > 0.0 ? weightedSum / node.totalMass : Vertex{0.0, 0.0, 0.0};
            node.size = 0.0;
            for (int d = 0; d < 3 && node.totalMass > 0.0; ++d) {
                node.size = std::max(node.size, node.boxMax[d] - node.boxMin[d]);
            }
        }
    }

    std::vector<kdtree::Vertex> Simulation::computeAccelerationsBarnesHut() {
        // topology only changes on a rebuild, mass/com/boxes change every step
        if (updateTree() || _barnesHutNodes.empty()) {
            flattenTree();
        }
        updateBarnesHutNodes();

        const double thetaSquared = _config.barnesHutTheta * _config.barnesHutTheta;
        std::vector<Vertex> accelerations(_positions.size(), Vertex{0.0, 0.0, 0.0});
        std::vector<int> stack{};
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            const Vertex &position = _positions[i];
            Vertex acceleration{0.0, 0.0, 0.0};
            stack.assign(1, 0);
            while (!stack.empty()) {
                const auto &node = _barnesHutNodes[stack.back()];
                stack.pop_back();
                if (node.totalMass <= 0.0) {
                    continue;
                }
                if (node.children[0] < 0) {
                    // leaf: few particles, just sum them directly (skipping ourselves)
                    for (const auto other: node.members) {
                        if (other != i) {
                            acceleration = acceleration + pairAcceleration(position, _positions[other], _masses[other]);
                        }
                    }
                    continue;
                }
                const Vertex delta = node.centerOfMass - position;
                const double distanceSquared = dot(delta, delta);
                // never approximate a node we're inside of, its com contains our own mass
                // (otherwise happens for theta >= 1/sqrt(3) and we'd pull on ourselves)
                bool containsPosition = true;
                for (int d = 0; d < 3 && containsPosition; ++d) {
                    containsPosition = node.boxMin[d] <= position[d] && position[d] <= node.boxMax[d];
                }
                // opening criterion size / distance < theta, squared to skip the sqrt
                if (!containsPosition && node.size * node.size < thetaSquared * distanceSquared) {
                    acceleration = acceleration + pairAcceleration(position, node.centerOfMass, node.totalMass);
                    continue;
                }
                for (const auto childIndex: node.children) {
                    if (childIndex >= 0) {
                        stack.push_back(childIndex);
                    }
                }
            }
            accelerations[i] = acceleration;
        }
        return accelerations;
    }

    std::vector<kdtree::Vertex> Simulation::computeAccelerations(const std::vector<Cluster> &clusters) const {
        std::vector<Vertex> accelerations(_positions.size(), Vertex{0.0, 0.0, 0.0});

        for (const auto &cluster: clusters) {
            // Particles sharing a leaf node are close together, so their interactions are
            // computed directly rather than approximated. By Newton's third law, a pair's
            // acceleration contributions are equal in magnitude and opposite in direction
            // (scaled by the other particle's mass), so each pair only needs to be evaluated
            // once instead of twice; this halves the direct-interaction work. This is safe
            // without synchronization only as long as this loop stays single-threaded, since
            // each iteration writes into two shared accelerations[] slots.
            for (std::size_t a = 0; a < cluster.members.size(); ++a) {
                const auto &[indexA, positionA] = cluster.members[a];
                for (std::size_t b = a + 1; b < cluster.members.size(); ++b) {
                    const auto &[indexB, positionB] = cluster.members[b];
                    const Vertex delta = positionB - positionA;
                    const double distanceSquared = dot(delta, delta) + _config.softening * _config.softening;
                    const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
                    const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
                    const Vertex accelerationPerUnitMass = delta * (_config.gravitationalConstant * inverseDistanceCubed);
                    accelerations[indexA] = accelerations[indexA] + accelerationPerUnitMass * _masses[indexB];
                    accelerations[indexB] = accelerations[indexB] - accelerationPerUnitMass * _masses[indexA];
                }
            }

            // Every other cluster/leaf is approximated as a single combined point mass
            // located at that leaf's center of mass.
            for (const auto &[particleIndex, particlePosition]: cluster.members) {
                for (const auto &otherCluster: clusters) {
                    if (&otherCluster == &cluster) {
                        continue;
                    }
                    accelerations[particleIndex] = accelerations[particleIndex] + accelerationTowardsCluster(particlePosition, otherCluster);
                }
            }
        }
        return accelerations;
    }

    std::vector<kdtree::Vertex> Simulation::computeAccelerationsBruteForce() const {
        std::vector<Vertex> accelerations(_positions.size(), Vertex{0.0, 0.0, 0.0});
        // By Newton's third law, a pair's acceleration contributions are equal in magnitude
        // and opposite in direction (scaled by the other particle's mass), so each pair only
        // needs to be evaluated once instead of twice; this halves the O(n^2) direct-sum work.
        for (std::size_t i = 0; i < _positions.size(); ++i) {
            for (std::size_t j = i + 1; j < _positions.size(); ++j) {
                const Vertex delta = _positions[j] - _positions[i];
                const double distanceSquared = dot(delta, delta) + _config.softening * _config.softening;
                const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
                const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
                const Vertex accelerationPerUnitMass = delta * (_config.gravitationalConstant * inverseDistanceCubed);
                accelerations[i] = accelerations[i] + accelerationPerUnitMass * _masses[j];
                accelerations[j] = accelerations[j] - accelerationPerUnitMass * _masses[i];
            }
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
