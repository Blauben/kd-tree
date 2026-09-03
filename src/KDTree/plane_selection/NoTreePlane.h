#pragma once

#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/SplitParam.h"


namespace kdtree {

    class NoTreePlane final : public PlaneSelectionAlgorithm {
        /**
        * Builds a plane with infinite cost. That way splitting is considered unpractical and no tree is built -> intersection is performed directly on the trinagle shapes.
        * @param splitParam specifies the space section to be split @link SplitParam.
        * @return Tuple of the default plane to split the specified bounding box, infinite cost as double and a list of empty shape sets. Refer to {@link ObjectIndexVectors<2>} for more information.
        */
        std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> findPlane(const SplitParam &splitParam) override {
            return {Plane{}, std::numeric_limits<double>::infinity(), ObjectIndexVectors<2>{std::make_unique<ObjectIndexVector>(), std::make_unique<ObjectIndexVector>()}};
        }
    };
}// namespace kdtree
