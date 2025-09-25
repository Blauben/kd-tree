#pragma once

#include "KDTree/tree/SplitParam.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"


namespace kdtree {

    class NoTreePlane final : public PlaneSelectionAlgorithm {
        /**
        * Builds a plane with infinite cost. That way splitting is considered unpractical and no tree is built -> intersection is performed directly on the trinagle faces.
        * @param splitParam specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the default plane to split the specified bounding box, infinite cost as double and a list of empty triangle sets. Refer to {@link PointIndexVectors<2>} for more information.
        */
        std::tuple<Plane, double, PointIndexVectors<2>> findPlane(const SplitParam &splitParam) override {
            return {Plane{}, std::numeric_limits<double>::infinity(), PointIndexVectors<2>{std::make_unique<PointIndexVector>(), std::make_unique<PointIndexVector>()}};
        }
    };
}// namespace kdtree