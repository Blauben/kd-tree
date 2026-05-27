#pragma once

#include "KDTree/util/UtilityContainer.h"
#include "KDTree/util/Constants.h"
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
     * Concept for a Vertex-like type: any indexed, sized container
     * providing three elements convertible to double.
     */
    template<typename T>
    concept VertexLike = requires(T v) {
        { v[0] } -> std::convertible_to<double>;
        { v[1] } -> std::convertible_to<double>;
        { v[2] } -> std::convertible_to<double>;
    };

    // Concrete internal vertex type used throughout the implementation
    using Vertex = std::array<double, 3>;

    /**
     * Allow KDTree data to be dynamic or static to avoid dangling references.
     */
    using VertexHandle = std::variant<Vertex, const Vertex*>;

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
                return {1, 0, 0};
            case Direction::Y:
                return {0, 1, 0};
            case Direction::Z:
                return {0, 0, 1};
            default:
                throw std::invalid_argument{"Unknown Direction enum value used during normal fetching."};
        }
    }

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

    template<typename T>
    const Vertex& asVertex(const T &object) {
        if constexpr (std::is_same_v<T, VertexHandle>) {
            // Handle variant: extract either Vertex or pointer to it
            if (const auto* ptr = std::get_if<const Vertex*>(&object)) {
                return **ptr;
            }
            return std::get<Vertex>(object);
        } else if constexpr (std::is_pointer_v<T>) {
            // Handle raw pointers
            return *object;
        } else {
            // Direct value
            return object;
        }
    }

    /**
    * Calculates the min and max coordinate values for each dimension of the elements supplied.
    * @param elements the container of whose elements to search for min and max ccordinates
    * @return the findings formatted in a pair of new elements. E.g <(0,0,0) , (1,1,1)> if the container {(0,0,1), (1,1,0)} is passed.
    */
    template<util::Container C>
    std::pair<Vertex, Vertex> findMinMaxCoordinates(C elements)
        requires std::is_same_v<typename C::value_type, Vertex> || std::is_same_v<typename C::value_type, VertexHandle> {
        //return empty box centered at the origin if no vertices provided
        if (elements.empty()) {
            return {Vertex{0, 0, 0}, Vertex{0, 0, 0}};
        }
        //initialize values from the array -> even if only one vertex is provided the box is still correct without executing the loop.
        Vertex min = asVertex(elements[0]);
        Vertex max = asVertex(elements[0]);

        //test each vertex for proximity to the origin and find minima and maxima
        for (const auto &vertex: elements) {
            const auto &coords = asVertex(vertex);
            // test each dimension separately
            for (size_t i = 0; i < coords.size(); ++i) {
                min[i] = std::min(min[i], coords[i]);
                max[i] = std::max(max[i], coords[i]);
            }
        }
        return {min, max};
    }
}// namespace kdtree