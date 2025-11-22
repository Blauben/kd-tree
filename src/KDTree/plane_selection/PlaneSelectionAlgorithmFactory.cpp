#include "KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"

namespace kdtree {
    std::shared_ptr<PlaneSelectionAlgorithm> PlaneSelectionAlgorithmFactory::create(const PlaneSelectionAlgorithm::Algorithm algorithm) {
        using Algorithm = PlaneSelectionAlgorithm::Algorithm;
        switch (algorithm) {
            case Algorithm::NOTREE:
                return std::make_shared<NoTreePlane>();
            case Algorithm::QUADRATIC:
                return std::make_shared<SquaredPlane>();
            case Algorithm::LOGSQUARED:
                return std::make_shared<LogNSquaredPlane>();
            case Algorithm::LOG:
                return std::make_shared<LogNPlane>();
            default:
                throw std::runtime_error("Unknown algorithm");
        }
    }
}// namespace kdtree
