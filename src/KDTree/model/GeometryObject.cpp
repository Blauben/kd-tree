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
    std::vector<Array3> GeometryObject::getVertices() const {
        std::vector<Array3> vertexCoords{};
        vertexCoords.reserve(objVertices.size());
        for (size_t i = 0; i < objVertices.size(); ++i) {
            vertexCoords.emplace_back(this->operator[](i));
        }
        return vertexCoords;
    }
}// namespace kdtree