//
// Created by saruman on 06.11.25.
//
#pragma once
#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {
    class GeometryObject {
    public:
        static size_t runningIndex;
        static const std::vector<Array3> vertices;

        const size_t index;
        const std::vector<size_t> objVertices;
        explicit GeometryObject(const std::vector<size_t>& objVertices);
    private:
    };
};
