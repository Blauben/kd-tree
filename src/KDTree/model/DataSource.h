#pragma once

#include "polyhedralGravity/model/Polyhedron.h"
#include <tuple>

namespace kdtree {

/**
 * Interface consisting of a method which returns a polyhedron.
 */
    class DataSource {

    public:

        /**
         * Default destructor of DataSource
         */
        virtual ~DataSource() = default;

        /**
         * Returns a Polyhedron from the underlying source.
         * @return a polyhedron
         */
        virtual std::tuple<std::vector<Array3>, std::vector<IndexArray3>> getPolyhedralSource() = 0;

    };

}
