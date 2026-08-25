#include "KDTree/model/GeometryObject.h"

#include <utility>

namespace kdtree {
    GeometryObject::GeometryObject(IndexVector objVertices, const size_t objIndex, const std::shared_ptr<std::vector<VertexHandle>> &vertices)
        : objIndex{objIndex}, objVertices{std::move(objVertices)}, _vertices{vertices} {
    }

    Vertex GeometryObject::operator[](const size_t index) const {
        const auto &vertices = *_vertices;
        return std::visit(util::overloaded{
                                  [&](const Vertex *vertex) { return *vertex; },
                                  [&](const Vertex &vertex) { return vertex; }},
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
