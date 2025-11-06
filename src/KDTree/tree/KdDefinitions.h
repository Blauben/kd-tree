#pragma once

#include "KDTree/model/GeometryObject.h"
#include "KDTree/util/UtilityContainer.h"
#include "thrust/detail/execution_policy.h"
#include "thrust/execution_policy.h"
#include "thrust/system/detail/sequential/for_each.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/iterator/transform_iterator.h>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace kdtree {

    /**
     * Alias for an array of size 3 (double)
     * @example for x, y, z coordinates.
     */
    using Array3 = std::array<double, 3>;

    /**
     * Alias for an array of size 3 (size_t)
     * @example for the vertex indices in a triangular face.
     */
    using IndexVector = std::vector<size_t>;

    /**
     * Alias for a triplet of arrays of size 3
     * @example for the segment of a triangular face
     */
    using Array3Triplet = std::array<Array3, 3>;

    /**
     * Assigns an integer index to the coordinate axes
     *
     * Used to specify coordinates. E.g. CoordinateArray[Direction]
     */
    enum class Direction {
        X = 0,
        Y = 1,
        Z = 2
    };

    static const std::vector ALL_DIRECTIONS{Direction::X, Direction::Y, Direction::Z};


    /**
     * Returns the normal vector for a direction.
     * @param direction The direction to return the normal vector for.
     * @return The normal vector.
     */
    static Array3 normal(Direction direction);

    /**
     * Number of dimensions for the polyhedron. Also corresponds to the number of elements of the {@link Direction} enum.
     */
    constexpr int DIMENSIONS = 3;

    /**
     * A set that stores indices of the faces vector in the KDTree. This effectively corresponds to a set of triangles. For performance purposes a std::vector is used instead of a std::set.
     */
    using ObjectIndexVector = std::vector<size_t>;

    /**
    * Triangle sets contained in an array. Used by the KDTree to divide a bounding boxes included triangles into smaller subsets. For the semantic purpose of the contained sets please refer to the comments in the usage context.
     */
    template<size_t Number>
    using TriangleIndexVectors = std::array<std::unique_ptr<ObjectIndexVector>, Number>;

    /**
    * Used by {@link PlaneEvent} to position the face that generated the event relative to the generated plane.
    */
    enum class PlaneEventType {
        ending = 0,
        planar = 1,
        starting = 2,
    };

    /**
     * Generated when traversing the vector of faces and building their candidate planes.
     */
    struct PlaneEvent {
        PlaneEventType type;
        /**
         * The candidate plane suggested by the face included in this struct.
         */
        Plane plane;
        /**
         * The index of the face that generated this candidate plane.
         */
        unsigned int faceIndex;

        PlaneEvent(PlaneEventType type, Plane plane, unsigned faceIndex);

        PlaneEvent() = default;

        /**
         * Less operator used for sorting an PlaneEvent vector.
         * @param other the PlaneEvent to compare this to.
         * @return true if this should precede the other argument.
         */
        bool operator<(const PlaneEvent &other) const;

        /**
         *Equality operator used for testing purposes
         */
        bool operator==(const PlaneEvent &other) const;
    };

    /**
     * A list of PlaneEvents.
    */
    using PlaneEventVector = std::vector<PlaneEvent>;

    /**
    * An array of PlaneEventLists.
    */
    template<size_t Number>
    using PlaneEventVectors = std::array<std::unique_ptr<PlaneEventVector>, Number>;

    /**
     * Extracts the indices of the triangle faces that are referenced by the given PlaneEvents.
     * @param events The PlaneEvents containing information about the faces.
     * @return A list of face indices.
     */
    ObjectIndexVector convertEventsToFaces(const std::variant<ObjectIndexVector, PlaneEventVector> &events);

    size_t countFaces(const std::variant<ObjectIndexVector, PlaneEventVector> &triangles);

    /**
    * The distance to the tree's root from this node. Used to limit the depth and the size of the tree.
    * @param nodeId The id of the node to determine the depth (distance to root node) of.
    */
    size_t recursionDepth(size_t nodeId);

}// namespace kdtree