#pragma once

#include <cstddef>
#include <vector>

#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {
    /**
     * This class contains a collection of vertices, representing the corners of a geometrical shape and provides the abstraction on which the kdtree operates.
     */
    class GeometryObject {
    public:
        /**
         * This field stores the index of the next GeometryObject and is increased whenever a new Object is increased.
         */
        static size_t runningIndex;
        /**
         * The global vertices of the scene on which the GeometryObjects are built on.
         */
        static std::vector<Vertex> vertices;

        /**
         * This index uniquely identifies a GeometryObject during kdtree construction.
         */
        const size_t objIndex;

        /**
         * Stores information on the corner vertices of this GeometryObject by storing their respective indices referencing the static vertices.
         */
        const IndexVector objVertices;
        explicit GeometryObject(const IndexVector& objVertices);

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
    };

    /**
        * An iterator transforming face indices to vertices and returning both.
        * This function returns a pair of transform iterators (first = begin(), second = end()).
        * @param begin begin iterator of the face indice vector to transform.
        * @param end end iterator of the face indice vector to transform.
        * @param vertices the vector of vertices to look up the indices obtained from the faces vector.
        * @param objectIndices the faces vector to lookup face indices.
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
};
