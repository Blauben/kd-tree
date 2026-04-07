#include "KDTree/model/GeometryObject.h"

#include <utility>

namespace kdtree {
    GeometryObject::GeometryObject(IndexVector objVertices) : objIndex{runningIndex++}, objVertices{std::move(objVertices)} {
    }

    //static initialization
    size_t GeometryObject::runningIndex{0};

    std::vector<VertexHandle> GeometryObject::vertices;

    Vertex GeometryObject::operator[](const size_t index) const {
        return std::visit(util::overloaded{
                [&](const Vertex *vertex) { return *vertex; },
                [&](const Vertex &vertex) { return vertex; }
            },
            vertices[objVertices[index]]);
    }

    const IndexVector &GeometryObject::getIndexVector() const {
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