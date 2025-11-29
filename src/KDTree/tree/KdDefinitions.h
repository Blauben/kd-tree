#pragma once

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
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace kdtree {

    /**
     * Alias for an array of size 3 (double), representing a vertex
     * @example for x, y, z coordinates.
     */
    using Vertex = std::array<double, 3>;

    /**
     * Alias for an array of size 3 (size_t)
     * @example for the vertex indices in a triangular face.
     */
    using IndexVector = std::vector<size_t>;

    /**
     * Alias for a triplet of arrays of size 3
     * @example for the segment of a triangular face
     */
    using VertexTriplet = std::array<Vertex, 3>;

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

    /**
     * A vector containing all directions.
     */
    static const std::vector ALL_DIRECTIONS{Direction::X, Direction::Y, Direction::Z};

    /**
     * Overloads the output stream operator for Direction enum class.
     * @param os The output stream.
     * @param direction The Direction enum value to be printed.
     * @return The output stream with the Direction value appended.
     */
    std::ostream &operator<<(std::ostream &os, const Direction &direction);


    /**
     * Returns the normal vector for a direction.
     * @param direction The direction to return the normal vector for.
     * @return The normal vector.
     */
    inline Vertex normal(const Direction direction) {
        switch (direction) {
            case Direction::X:
                return Vertex{1, 0, 0};
            case Direction::Y:
                return Vertex{0, 1, 0};
            case Direction::Z:
                return Vertex{0, 0, 1};
            default:
                throw std::invalid_argument{"Unknown Direction enum value used during normal fetching."};
        }
    }

    /**
     * Number of dimensions for the polyhedron. Also corresponds to the number of elements of the {@link Direction} enum.
     */
    constexpr int DIMENSIONS = 3;

    /**
     * A set that stores indices of the object vector in the KDTree. This effectively corresponds to a set of shapes. For performance purposes a std::vector is used instead of a std::set.
     */
    using ObjectIndexVector = std::vector<size_t>;

    /**
    * Shape sets contained in an array. Used by the KDTree to divide a bounding boxes included shapes into smaller subsets. For the semantic purpose of the contained sets please refer to the comments in the usage context.
     */
    template<size_t Number>
    using ObjectIndexVectors = std::array<std::unique_ptr<ObjectIndexVector>, Number>;

    /**
    * The distance to the tree's root from this node. Used to limit the depth and the size of the tree.
    * @param nodeId The id of the node to determine the depth (distance to root node) of.
    */
    size_t recursionDepth(size_t nodeId);

}// namespace kdtree