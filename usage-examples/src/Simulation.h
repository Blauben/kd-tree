// Gravity particle simulation built on top of the KDTree library.
//
// This example clusters particles by KDTree leaf node: each leaf's particles are
// collapsed into a single point mass (center of mass, combined mass). Forces
// between particles in the same leaf are computed directly (brute force), while
// forces from particles in other leaves are approximated using that leaf's
// combined point mass. All particles move under gravity every step, so by default
// the tree is kept in sync via KDTree::rebuildTreeIfNeeded(), which only performs a
// full rebuild once a particle has actually moved outside its leaf's bounding box
// instead of unconditionally rebuilding every step.
#pragma once

#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <cstddef>
#include <vector>

namespace gravity_demo {

    struct SimulationConfig {
        std::size_t particleCount = 200;
        double halfExtent = 10.0;
        double particleMass = 1.0;
        double gravitationalConstant = 1.0;
        double softening = 0.05;
        double timeStep = 0.01;
        double initialSpin = 0.4;
        unsigned int seed = 42;
        kdtree::PlaneSelectionAlgorithm::Algorithm algorithm = kdtree::PlaneSelectionAlgorithm::Algorithm::LOG;
        /**
         * If true, uses KDTree::rebuildTreeIfNeeded() each step, which skips the rebuild
         * entirely when no particle has moved outside its leaf node's bounding box since
         * the last build. If false, KDTree::rebuildTree() is called unconditionally every
         * step instead.
         */
        bool adaptiveRebuild = true;
        /**
         * If true (default), forces are computed via the KDTree leaf-cluster approximation
         * described above. If false, the KDTree is never built or queried at all, and forces
         * are instead computed with a brute-force O(n^2) pairwise sum over every particle
         * pair, so the two modes' timings can be compared directly.
         */
        bool useKdTree = true;
    };

    /**
     * Drives an N-body gravity simulation, using a KDTree to cluster nearby particles
     * (by leaf node) so that far-away interactions can be approximated with a single
     * combined point mass instead of a full pairwise sum.
     */
    class Simulation {
    public:
        explicit Simulation(const SimulationConfig &config);

        /**
         * Advances the simulation by one time step using kick-drift-kick leapfrog integration.
         */
        void step();

        [[nodiscard]] const std::vector<kdtree::Vertex> &positions() const {
            return _positions;
        }
        [[nodiscard]] const std::vector<kdtree::Vertex> &velocities() const {
            return _velocities;
        }
        [[nodiscard]] const std::vector<double> &masses() const {
            return _masses;
        }
        [[nodiscard]] double totalKineticEnergy() const;
        [[nodiscard]] std::size_t lastClusterCount() const {
            return _lastClusterCount;
        }
        /**
         * Whether the most recent call to step() actually rebuilt the KDTree. Always true
         * when SimulationConfig::adaptiveRebuild is false. When it is true, this reports
         * whether KDTree::rebuildTreeIfNeeded() decided a rebuild was necessary.
         */
        [[nodiscard]] bool lastRebuildOccurred() const {
            return _lastRebuildOccurred;
        }

        /**
         * Prints the underlying kd-tree to console. Useful for debugging.
         */
        void printKdTree();

    private:
        /**
         * A single KDTree leaf node collapsed into its combined mass and center of mass,
         * together with the original particles it contains (for direct in-leaf forces).
         */
        struct Cluster {
            kdtree::Vertex centerOfMass{};
            double totalMass = 0.0;
            std::vector<std::pair<size_t, kdtree::Vertex>> members{};
        };

        [[nodiscard]] std::vector<Cluster> buildClusters();
        [[nodiscard]] std::vector<kdtree::Vertex> computeAccelerations(const std::vector<Cluster> &clusters) const;
        [[nodiscard]] kdtree::Vertex accelerationTowardsCluster(const kdtree::Vertex &position, const Cluster &cluster) const;
        /**
         * Computes every particle's acceleration via a direct O(n^2) pairwise sum, without
         * touching the KDTree at all. Used by SimulationConfig::useKdTree = false to compare
         * performance against the leaf-cluster approximation.
         */
        [[nodiscard]] std::vector<kdtree::Vertex> computeAccelerationsBruteForce() const;
        [[nodiscard]] std::vector<kdtree::Vertex> computeNextAccelerations();

        SimulationConfig _config;
        std::vector<kdtree::Vertex> _positions;
        std::vector<kdtree::Vertex> _velocities;
        std::vector<double> _masses;
        std::vector<kdtree::Vertex> _accelerations;
        kdtree::KDTree _tree;
        std::size_t _lastClusterCount = 0;
        bool _lastRebuildOccurred = true;
    };

}// namespace gravity_demo
