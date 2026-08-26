#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "KDTree/plane_selection/PlaneEventAlgorithm.h"
#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/util/pragma.h"
#include "thrust/detail/execution_policy.h"
#include "thrust/execution_policy.h"
#include "thrust/system/detail/sequential/for_each.h"

#if defined(KD_TREE_TBB)
#include <tbb/combinable.h>
#include <tbb/parallel_for.h>
#endif
#if defined(KD_TREE_OMP)
#include <omp.h>
#endif

namespace kdtree {
    //forward declaration
    struct SplitParam;

    /**
     * A strategy for calculating optimal planes. Part of the Strategy Software pattern.
     */
    class LogNSquaredPlane final : public PlaneEventAlgorithm {
        using OptimalPlaneLogNSquared = OptimalPlane<std::shared_ptr<PlaneEventVector>, const bool>;

    public:
        std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> findPlane(
                const SplitParam &splitParam) override;

    private:
        /**
         * Generates the optimal split plane considering a single dimension.
         * @param optPlane
         * @return the optimal plane, its cost, the events that were generated in the process, and whether to include planar shapes in the minimal bounding box.
         */
        static void findPlaneForSingleDimension(OptimalPlaneLogNSquared &optPlane);


        /**
        * When an optimal plane has been found extract the index lists of shapes for further subdivision through child nodes.
        * @param planeEvents The events that were generated during {@link findPlane}.
        * @param plane The plane to split the shapes by.
        * @param minSide Whether to include planar shapes to the bounding box closer to the origin.
        * @return The shapeIndexlists for the bounding boxes closer and further away from the origin.
        */
        static ObjectIndexVectors<2> generateGeometrySubsets(const std::shared_ptr<PlaneEventVector> &planeEvents, const Plane &plane,
                                                             bool minSide);
    };
}// namespace kdtree
