//
// Created by saruman on 06.11.25.
//
#pragma once

#include <cstddef>
#include <vector>

#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {
    class GeometryObject {
    public:
        static size_t runningIndex;
        static const std::vector<Array3> vertices;

        const size_t objIndex;
        const std::vector<size_t> objVertices;
        explicit GeometryObject(const std::vector<size_t>& objVertices);
        Array3 operator[](size_t index) const;
    private:
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
    [[nodiscard]] static auto transformIterator(const std::vector<size_t>::const_iterator begin, const std::vector<size_t>::const_iterator end, const std::vector<GeometryObject> &geometryObjects) {
        //The offset must be captured by value to ensure its lifetime!
        const auto lambdaApplication = [&geometryObjects](size_t objIndex) {
            const auto &object = geometryObjects[objIndex];
            Array3Triplet vertexTriplet = {
                object[0],
                object[1],
                object[2]
            };
            return std::make_pair(objIndex, vertexTriplet);
        };

        auto first = thrust::make_transform_iterator(begin, lambdaApplication);
        auto last = thrust::make_transform_iterator(end, lambdaApplication);
        return std::make_pair(first, last);
    }
};
