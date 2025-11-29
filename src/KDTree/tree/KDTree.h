#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <ostream>
#include <set>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <utility>
#include <vector>

#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/model/GeometryObject.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"
#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/LeafNode.h"
#include "KDTree/tree/SplitNode.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/tree/TreeNode.h"
#include "KDTree/tree/TreeNodeFactory.h"
#include "KDTree/util/UtilityContainer.h"

namespace kdtree {
    /**
     * A KDTree for a given polyhedron to speed up ray intersections with the polyhedron. It is thread safe.
     */
    class KDTree {
        /**
         * friend declaration for testing purposes.
         */
        friend class KDTreeTest_AlgorithmRegressionTest_Test;

        /**
        * The entry node of the KDTree. Only access using getter.
        */
        std::shared_ptr<TreeNode> _rootNode;

        /**
         * The polyhedron's vertices.
         */
        std::vector<GeometryObject> _geometryObjects;

        /**
         * Set when the root node has been created.
         */
        std::once_flag _rootNodeCreated;

        /**
        * Parameters for lazily building the root node {@link SplitParam}
        */
        std::unique_ptr<SplitParam> _splitParam;

    public:
        /**
        * Call to build a KDTree to speed up intersections of rays with a polyhedron's shapes.
        * @param vertices The vertex coordinates of the polyhedron
        * @param shapes The shapes of the polyhedron with a shape being a triplet of vertex indices
        * @param algorithm Specifies which algorithm to use for finding optimal split planes.
        * @return the lazily built KDTree.
        */
        KDTree(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &shapes,
               PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG);

        explicit KDTree(const std::vector<Vertex> &particles, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG);

        /**
         * Call to build a KDTree to speed up intersections of rays with a polyhedron's shapes.
         * @param nodeFilePath The path to the .node file containing information about the polyhedron's vertices.
         * @param faceFilePath The path to the .face file containing information about the polyhedron's shapes.
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        KDTree(const std::string &nodeFilePath, const std::string &faceFilePath, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG);

        /**
         * Constructor overload that allows passing the paths for the .node and .face files in a std::pair.
         * @param polySource The pair of the .node and .face file
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        KDTree(const std::tuple<std::vector<Vertex>, std::vector<IndexVector>> &polySource,
               PlaneSelectionAlgorithm::Algorithm algorithm);


        /**
        * Creates the root tree node if not initialized and returns it.
        * @return the root tree Node.
        */
        std::shared_ptr<TreeNode> getRootNode();

        /**
        * Used to calculate intersections of a ray and the polyhedron's shapes contained in this node.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @param intersections The set found intersection points are added to.
        */
        void getIntersections(const Vertex &origin, const Vertex &ray, std::set<Vertex> &intersections);

        /**
         * Calculates the number of intersections of a ray with the polyhedron.
         * @param origin The origin point of the ray.
         * @param ray The ray direction vector.
         * @return the number of intersections.
         */
        size_t countIntersections(const Vertex &origin, const Vertex &ray);

        /**
         * Prebuilds the whole KDTree bypassing lazy loading entirely.
         */
        KDTree &prebuildTree();

        /**
         * Overloads the output stream operator to print a representation of the KDTree.
         * @param os The output stream.
         * @param kdTree The KDTree to be printed.
         * @return The output stream with the KDTree representation appended.
         */
        friend std::ostream &operator<<(std::ostream &os, const KDTree &kdTree);

        friend std::string to_string(const KDTree &kdTree);
    };
}// namespace kdtree
