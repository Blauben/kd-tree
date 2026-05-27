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

#include "NodeRegister.h"
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
#include "KDTree/tree/PlaneIterator.h"
#include "KDTree/Logging.h"

namespace kdtree {
    /**
     * A KDTree for given particles or shapes to for example speed up ray intersections or any other operation that requires * spatial partitioning. It is thread safe.
     */
    class KDTree {
        /**
         * friend declaration for testing purposes.
         */
        friend class KDTreeTest_AlgorithmRegressionTest_Test;
        friend class KDTreeTest_RebuildTreeTest_Test;

        /**
        * The entry node of the KDTree. Only access using getter.
        */
        std::shared_ptr<TreeNode> _rootNode;

        /**
         * The polyhedron's vertices.
         */
        std::shared_ptr<std::vector<VertexHandle>> _vertices;

        /**
         * The polyhedron's geometry objects.
         */
        std::vector<GeometryObject> _geometryObjects;

        /**
         * The algorithm to use during tree construction for finding optimal split planes. The algorithm is stored in the KDTree to ensure that the same algorithm is used for all splits during tree construction and rebuilding.
         */
        const PlaneSelectionAlgorithm::Algorithm _algorithm;

        /**
         * Used to avoid multiple threads building the root node at the same time when getRootNode is called for the first time by multiple threads. After the root node is built, this mutex is not used anymore.
         */
        std::mutex _rootNodeCreationMutex;

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
        * @param copyVertices If true, the vertices are copied and stored internally (static). If false the tree will only store pointers to the vertices, allowing the user to modify vertices and have the changes reflected in the tree, but also requiring the user to ensure that the vertices outlive the tree (dynamic).
        * @return the lazily built KDTree.
        */
        KDTree(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &shapes,
             PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG, bool copyVertices = true);

        
        /**
         * Constructor overload that allows passing the vertices as a different type than Vertex, as long as they are VertexLike. The constructor will convert the vertices to Vertex by static_casting the coordinates to double. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand. The copyVertices parameter is not available in this overload and defaults to true, as it is not possible to guarantee that the original vertex type outlives the tree when passing a different type.
         * @tparam V The vertex type, which must be VertexLike.
         * @param vertices The vertex coordinates of the polyhedron
         * @param shapes The shapes of the polyhedron with a shape being a triplet of vertex indices
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        template<kdtree::VertexLike V>
        requires std::is_same_v<V, Vertex>
        KDTree(const std::vector<V> &vertices, const std::vector<IndexVector> &shapes,
            PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG)
        : KDTree(static_cast<const std::vector<Vertex>&>(vertices), shapes, algorithm, true) {}

        /**
         * Constructor overload for VertexLike vertex types that are not Vertex. Converts the vertices to Vertex by static_casting the coordinates to double. The copyVertices parameter is not available in this overload and defaults to true, as it is not possible to guarantee that the original vertex type outlives the tree when passing a different type.
         * @tparam V The vertex type, which must be VertexLike.
         * @param vertices The vertex coordinates of the polyhedron
         * @param shapes The shapes of the polyhedron with a shape being a triplet of vertex indices
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        template<kdtree::VertexLike V>
        requires (!std::is_same_v<V, Vertex>)
        KDTree(const std::vector<V> &vertices, const std::vector<IndexVector> &shapes,
             PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG)
            : KDTree([&]{
                std::vector<Vertex> converted{};
                converted.reserve(vertices.size());
                for (const auto &v : vertices) converted.emplace_back(Vertex{static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2])});
                return converted;
            }(), shapes, algorithm, true) {}

        /**
         * Constructor overload for building a KDTree with particles instead of shapes. Each particle is represented by a single vertex and the tree will not contain any shapes. This means that the tree can be used to efficiently find the nearest point on a ray to a targeted vertex, but not for example to find ray-triangle intersections.
         * @see KDTree(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &shapes, PlaneSelectionAlgorithm::Algorithm algorithm) for more details on the parameters.
         */
        explicit KDTree(const std::vector<Vertex> &particles, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG);

        /**
         * Constructor overload for building a KDTree with particles instead of shapes. Each particle is represented by a single vertex and the tree will not contain any shapes. This means that the tree can be used to efficiently find the nearest point on a ray to a targeted vertex, but not for example to find ray-triangle intersections. This overload allows passing the vertices as a different type than Vertex, as long as they are VertexLike. The constructor will convert the vertices to Vertex by static_casting the coordinates to double. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand.
         * @tparam V The vertex type, which must be VertexLike.
         * @param particles The vertex coordinates of the particles
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         */
        template<kdtree::VertexLike V>
        requires std::is_same_v<V, Vertex>
        explicit KDTree(const std::vector<V> &particles, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG)
            : KDTree(static_cast<const std::vector<Vertex>&>(particles), algorithm) {}

        /**
         * Constructor overload for VertexLike vertex types that are not Vertex. Converts the vertices to Vertex by static_casting the coordinates to double. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand.
         * @tparam V The vertex type, which must be VertexLike.
         * @param particles The vertex coordinates of the particles
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         */
        template<kdtree::VertexLike V>
        requires (!std::is_same_v<V, Vertex>)
        explicit KDTree(const std::vector<V> &particles, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG)
            : KDTree([&]{ std::vector<Vertex> converted{}; converted.reserve(particles.size()); for (const auto &v: particles) converted.emplace_back(Vertex{static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2])}); return converted;}(), algorithm) {}

        /**
         * Call to build a KDTree to speed up intersections of rays with a polyhedron's shapes.
         * @param nodeFilePath The path to the .node file containing information about the polyhedron's vertices.
         * @param faceFilePath The path to the .face file containing information about the polyhedron's shapes.
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        KDTree(const std::string &nodeFilePath, const std::string &faceFilePath, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG);

        /**
         * Constructor overload that takes a tuple of vertices and shapes as input. This allows for example to pass the output of a TetgenAdapter directly to the KDTree constructor without having to unpack the tuple first. The copyVertices parameter is not available in this overload and defaults to true, as it is not possible to guarantee that the original vertex type outlives the tree when passing a tuple.
         * @param polySource A tuple containing the vertices and shapes of the polyhedron. The vertices are expected to be in the first element of the tuple and the shapes in the second element.
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         **/
         KDTree(std::tuple<std::vector<Vertex>, std::vector<IndexVector>> polySource,
             PlaneSelectionAlgorithm::Algorithm algorithm);

        /**
         * Constructor overload for VertexLike vertex types that are not Vertex. Converts the vertices to Vertex by static_casting the coordinates to double. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand. The copyVertices parameter is not available in this overload and defaults to true, as it is not possible to guarantee that the original vertex type outlives the tree when passing a tuple.
         * @tparam V The vertex type, which must be VertexLike.
         * @param polySource A tuple containing the vertices and shapes of the polyhedron. The vertices are expected to be in the first element of the tuple and the shapes in the second element.
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        template<kdtree::VertexLike V>
        requires std::is_same_v<V, Vertex>
        KDTree(std::tuple<std::vector<V>, std::vector<IndexVector>> polySource,
            PlaneSelectionAlgorithm::Algorithm algorithm)
            : KDTree(static_cast<std::tuple<std::vector<Vertex>, std::vector<IndexVector>>&>(polySource), algorithm) {}

        /**
         * Constructor overload for VertexLike vertex types that are not Vertex. Converts the vertices to Vertex by static_casting the coordinates to double. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand. The copyVertices parameter is not available in this overload and defaults to true, as it is not possible to guarantee that the original vertex type outlives the tree when passing a tuple.
         * @tparam V The vertex type, which must be VertexLike.
         * @param polySource A tuple containing the vertices and shapes of the polyhedron. The vertices are expected to be in the first element of the tuple and the shapes in the second element.
         * @param algorithm Specifies which algorithm to use for finding optimal split planes.
         * @return the lazily built KDTree.
         */
        template<kdtree::VertexLike V>
        requires (!std::is_same_v<V, Vertex>)
        KDTree(std::tuple<std::vector<V>, std::vector<IndexVector>> polySource,
            PlaneSelectionAlgorithm::Algorithm algorithm)
            : KDTree(std::tuple<std::vector<Vertex>, std::vector<IndexVector>>{
                [&]{ auto &vec = std::get<0>(polySource); std::vector<Vertex> c; c.reserve(vec.size()); for (const auto &v: vec) c.emplace_back(Vertex{static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2])}); return c;}(),
                std::get<1>(polySource)
            }, algorithm) {}


        /**
         * Container tracks handles to the tree nodes for direct access to the nodes, e.g. for checking if a tree rebuild is needed due to vertex movement outside the leaf node's bounding box. The handles are stored as weak_ptrs to avoid memory leaks and dangling pointers when tree nodes are destroyed.
         */
        NodeRegister nodeRegister{};

        /**
        * Creates the root tree node if not initialized and returns it.
        * @return the root tree Node.
        */
        std::shared_ptr<TreeNode> getRootNode();

        void rebuildTree();

        /**
        * Used to calculate intersections of a ray and the polyhedron's shapes contained in this node.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @param intersections The set found intersection points are added to.
        */
        void getIntersections(const Vertex &origin, const Vertex &ray, std::set<Vertex> &intersections);

        /**
         * Overload of getIntersections for VertexLike vertex types that are not Vertex. Converts the vertices to Vertex by static_casting the coordinates to double before calling the main getIntersections function. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand.
         */
        template<kdtree::VertexLike V>
        void getIntersections(const V &origin, const V &ray, std::set<Vertex> &intersections) {
            Vertex o{static_cast<double>(origin[0]), static_cast<double>(origin[1]), static_cast<double>(origin[2])};
            Vertex r{static_cast<double>(ray[0]), static_cast<double>(ray[1]), static_cast<double>(ray[2])};
            return getIntersections(o, r, intersections);
        }

        /**
         * Calculates the number of intersections of a ray with the polyhedron.
         * @param origin The origin point of the ray.
         * @param ray The ray direction vector.
         * @return the number of intersections.
         */
        size_t countIntersections(const Vertex &origin, const Vertex &ray);

        /**
         * Overload of countIntersections for VertexLike vertex types that are not Vertex. Converts the vertices to Vertex by static_casting the coordinates to double before calling the main countIntersections function. This allows for example to pass vertices with single precision (e.g. std::array<float, 3>) without having to convert them to double precision beforehand.
         */
        template<kdtree::VertexLike V>
        size_t countIntersections(const V &origin, const V &ray) {
            Vertex o{static_cast<double>(origin[0]), static_cast<double>(origin[1]), static_cast<double>(origin[2])};
            Vertex r{static_cast<double>(ray[0]), static_cast<double>(ray[1]), static_cast<double>(ray[2])};
            return countIntersections(o, r);
        }

        /**
         * Prebuilds the whole KDTree bypassing lazy loading entirely.
         */
        KDTree &prebuildTree();

        /**
         * Generates iterators for the GeometryObjects contained in the kdtree.
         * @return a pair of iterators to iterate over the geometry objects contained in the KDTree.
         */
        [[nodiscard]] std::pair<std::vector<GeometryObject>::iterator, std::vector<GeometryObject>::iterator>
        geometryIterator();

        /**
         * Generates iterators for the planes contained in the kdtree. As the planes are not stored in a container but rather generated on the fly during tree construction, the iterators are implemented as a custom iterator class that traverses the tree and generates the planes lazily.
         * @return a pair of iterators to iterate over all split planes of the kd-tree.
         */
        std::pair<PlaneIterator, PlaneIterator> planeIterator();

        /**
         * Checks if any of the leaf nodes in the tree require a rebuild due to vertices having moved outside their bounding boxes since the last tree build. If a rebuild is needed, the tree is rebuilt to maintain correct intersection results.
         */
        void rebuildTreeIfNeeded();

        /**
         * Overloads the output stream operator to print a representation of the KDTree.
         * @param os The output stream.
         * @param kdTree The KDTree to be printed.
         * @return The output stream with the KDTree representation appended.
         */
        friend std::ostream &operator<<(std::ostream &os, const KDTree &kdTree);

        friend std::string to_string(const KDTree &kdTree);

        friend class PlaneIterator;
    };
}// namespace kdtree
