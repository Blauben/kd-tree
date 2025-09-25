#pragma once

#include "KDTree/plane_selection/NoTreePlane.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/plane_selection/SquaredPlane.h"

#include <memory>


namespace kdtree {
    class PlaneSelectionAlgorithmFactory {
    public:
        /**
         * Factory method to return the plane finding selection algorithm specified by the enum parameter.
         * @param algorithm Specifies which algorithm to return.
         * @return The algorithm which executes the requested strategy.
         */
        static std::shared_ptr<PlaneSelectionAlgorithm> create(PlaneSelectionAlgorithm::Algorithm algorithm);
    };
}// namespace kdtree