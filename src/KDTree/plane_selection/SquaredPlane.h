#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/util/UtilityContainer.h"
#include "thrust/detail/execution_policy.h"
#include "thrust/execution_policy.h"
#include "thrust/iterator/iterator_facade.h"
#include "thrust/iterator/transform_iterator.h"
#include "thrust/system/detail/sequential/for_each.h"

namespace kdtree {
    //forward declaration
    struct SplitParam;

    /**
     * A strategy for calculating optimal planes. Part of the Strategy Software pattern.
    * O(N^2) time complexity.
    */
    class SquaredPlane final : public PlaneSelectionAlgorithm {
        using OptimalPlaneSquared = OptimalPlane<ObjectIndexVectors<3>, bool>;

        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param splitParam specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of shape sets with respective positions to the found plane. Refer to {@link ObjectIndexVectors<2>} for more information.
        */
        std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> findPlane(
                const SplitParam &splitParam) override;

        /**
        * Splits a section of a polyhedron into two bounding boxes and calculates the shape shape sets contained in the new bounding boxes.
        * @param splitParam specifies the polyhedron section to be split.
        * @param split the plane by which to split the polyhedron section.
        * @return Three shape lists contained in an array. Those being the set of shapes with non-zero area in the bounding box closer to the origin with respect to the split plane,
        * the set of shapes with non-zero area in the bounding box further away from the origin with respect to the split plane.
        * The set of shapes that lies on the plane.
        */
        static ObjectIndexVectors<3> containedShapes(const SplitParam &splitParam, const Plane &split);
    };
}// namespace kdtree
