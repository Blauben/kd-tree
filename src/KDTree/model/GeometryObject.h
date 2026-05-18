#pragma once

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include <thrust/iterator/transform_iterator.h>

#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {
    /**
     * This class contains a collection of vertices, representing the corners of a geometrical shape and provides the abstraction on which the kdtree operates.
     */
    class GeometryObject {
    public:
        /**
         * This index uniquely identifies a GeometryObject during kdtree construction.
         */
        const size_t objIndex;

        /**
         * Stores indices of this object's corner vertices in the owning KDTree vertex storage.
         */
        const IndexVector objVertices;
        GeometryObject(IndexVector objVertices, size_t objIndex, const std::shared_ptr<std::vector<VertexHandle>> &vertices);

        /**
         * Returns the index-th vertex of the shape represented by this GeometryObject.
         *
         * @param index The n-th node which to return
         * @return a vertex
         */
        Vertex operator[](size_t index) const;

        /**
         * Get the indices of the corner vertices.
         * @return the indices stored in a std::vector
         */
        [[nodiscard]] const IndexVector &getIndexVector() const;

        /**
         * Get the corner vertices.
         * @return a std::vector of Vertex objects
         */
        [[nodiscard]] std::vector<Vertex> getVertices() const;

    private:
        // Vertex storage owned by the KDTree instance that created this object.
        const std::shared_ptr<std::vector<VertexHandle>> _vertices;
    };

    /**
        * An iterator transforming shape indices to vertices and returning both.
        * This function returns a pair of transform iterators (first = begin(), second = end()).
        * @param begin begin iterator of the shape indice vector to transform.
        * @param end end iterator of the shape indice vector to transform.
        * @param geometryObjects the list of GeometryObjects to retrieve the vertices from.
        * @return pair of transform iterators.
        */
    [[nodiscard]] inline auto transformIterator(const std::vector<size_t>::const_iterator begin, const std::vector<size_t>::const_iterator end, const std::vector<GeometryObject> &geometryObjects) {
        //The offset must be captured by value to ensure its lifetime!
        const auto lambdaApplication = [&geometryObjects](size_t objIndex) {
            const auto &object = geometryObjects[objIndex];
            return std::make_pair(objIndex, object.getVertices());
        };

        auto first = thrust::make_transform_iterator(begin, lambdaApplication);
        auto last = thrust::make_transform_iterator(end, lambdaApplication);
        return std::make_pair(first, last);
    }
};// namespace kdtree
