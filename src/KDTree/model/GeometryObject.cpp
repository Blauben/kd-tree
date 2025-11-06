//
// Created by saruman on 06.11.25.
//
#include "KDTree/model/GeometryObject.h"

namespace kdtree {
    GeometryObject::GeometryObject(const std::vector<size_t>& objVertices) : index{runningIndex++}, objVertices{objVertices} {}
}