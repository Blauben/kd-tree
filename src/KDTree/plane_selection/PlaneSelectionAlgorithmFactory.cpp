#include "KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"

namespace kdtree {
    std::shared_ptr<PlaneSelectionAlgorithm> PlaneSelectionAlgorithmFactory::create(const PlaneSelectionAlgorithm::Algorithm algorithm) {
        using Algorithm = PlaneSelectionAlgorithm::Algorithm;
        switch (algorithm) {
            case Algorithm::NOTREE:
                return std::make_shared<NoTreePlane>();
            default:
            case Algorithm::QUADRATIC:
                return std::make_shared<SquaredPlane>();
        }
    }
}// namespace kdtree
