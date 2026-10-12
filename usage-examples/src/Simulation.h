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
//
// With SimulationConfig::barnesHut the whole tree hierarchy is used instead (Barnes-Hut):
// every node carries its combined mass/center of mass and is approximated as one point
// mass once it's far enough away, which gets the force calculation down to ~O(n log n).
#pragma once

#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <array>
#include <cstddef>
#include <iostream>
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
        /**
         * Only used with useKdTree. If true, forces are computed with a Barnes-Hut traversal
         * over the whole KDTree hierarchy instead of the flat leaf-cluster approximation:
         * every node (inner nodes too) carries its combined mass and center of mass, and a
         * node is treated as a single point mass once size / distance < barnesHutTheta.
         */
        bool barnesHut = false;
        /**
         * Barnes-Hut opening angle. Smaller is more accurate and slower, 0 degenerates to an
         * exact (but slower than brute force) traversal down to every leaf.
         */
        double barnesHutTheta = 0.5;
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

        /**
         * A KDTree node flattened into an array for the Barnes-Hut traversal. The topology
         * (children/members) only changes on a tree rebuild, mass/center of mass/size are
         * refreshed every step since particles move.
         */
        struct BarnesHutNode {
            std::array<int, 2> children{-1, -1};// -1 -> leaf
            std::vector<std::size_t> members{}; // particle indices, only filled for leaves
            double totalMass = 0.0;
            kdtree::Vertex centerOfMass{};
            double size = 0.0;// longest edge of the tight bounding box around the node's particles
            kdtree::Vertex boxMin{};
            kdtree::Vertex boxMax{};
        };

        /**
         * Rebuilds the KDTree if needed (or always, without adaptive rebuilding) and updates
         * _lastRebuildOccurred. Returns whether the tree changed.
         */
        bool updateTree();
        [[nodiscard]] std::vector<Cluster> buildClusters();
        /**
         * Copies the KDTree hierarchy into _barnesHutNodes in pre-order, so children always come
         * after their parent and the upward pass is a simple reverse loop.
         */
        void flattenTree();
        /**
         * Upward pass: recomputes mass, center of mass and bounding box of every node from the
         * current particle positions in O(n).
         */
        void updateBarnesHutNodes();
        [[nodiscard]] std::vector<kdtree::Vertex> computeAccelerationsBarnesHut();
        [[nodiscard]] kdtree::Vertex pairAcceleration(const kdtree::Vertex &position, const kdtree::Vertex &source, double sourceMass) const;
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
        std::vector<BarnesHutNode> _barnesHutNodes;
        std::size_t _lastClusterCount = 0;
        bool _lastRebuildOccurred = true;
    };

}// namespace gravity_demo
