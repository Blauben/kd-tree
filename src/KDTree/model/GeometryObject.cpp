//
// Created by saruman on 06.11.25.
//
#include "KDTree/model/GeometryObject.h"

namespace kdtree {
    GeometryObject::GeometryObject(const std::vector<size_t>& objVertices) : objIndex{runningIndex++}, objVertices{objVertices} {
    }
    Array3 GeometryObject::operator[](const size_t index) const {
        return vertices[objVertices[index]];
    }
}// namespace kdtree