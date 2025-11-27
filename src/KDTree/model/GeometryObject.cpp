#include "KDTree/model/GeometryObject.h"

namespace kdtree {
    GeometryObject::GeometryObject(const IndexVector& objVertices) : objIndex{runningIndex++}, objVertices{objVertices} {
    }

    //static initialization
    size_t GeometryObject::runningIndex{0};

    std::vector<std::array<double, 3>> GeometryObject::vertices;

    Vertex GeometryObject::operator[](const size_t index) const {
        return GeometryObject::vertices[objVertices[index]];
    }
    const IndexVector& GeometryObject::getIndexVector() const {
        return objVertices;
    }
    std::vector<Vertex> GeometryObject::getVertices() const {
        std::vector<Vertex> vertexCoords{};
        vertexCoords.reserve(objVertices.size());
        for (size_t i = 0; i < objVertices.size(); ++i) {
            vertexCoords.emplace_back(this->operator[](i));
        }
        return vertexCoords;
    }
}// namespace kdtree