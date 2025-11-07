//
// Created by saruman on 06.11.25.
//
#include "KDTree/model/GeometryObject.h"

namespace kdtree {
    GeometryObject::GeometryObject(const IndexVector& objVertices) : objIndex{runningIndex++}, objVertices{objVertices} {
    }

    size_t GeometryObject::runningIndex{0};

    std::vector<std::array<double, 3>> GeometryObject::vertices;

    Array3 GeometryObject::operator[](const size_t index) const {
        return GeometryObject::vertices[objVertices[index]];
    }
    const IndexVector& GeometryObject::getIndexVector() const {
        return objVertices;
    }
    Array3Triplet GeometryObject::getVertices() const {
        return {
            this->operator[](0),
            this->operator[](1),
            this->operator[](2)
        };
    }
}// namespace kdtree