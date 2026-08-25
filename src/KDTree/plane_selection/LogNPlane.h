#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "KDTree/plane_selection/PlaneEventAlgorithm.h"
#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/util/UtilityContainer.h"
#include "thrust/detail/execution_policy.h"
#include "thrust/execution_policy.h"
#include "thrust/iterator/iterator_facade.h"
#include "thrust/iterator/transform_iterator.h"
#include "thrust/system/detail/sequential/for_each.h"

namespace kdtree {
    struct SplitParam;

    /**
     * A strategy for calculating optimal planes. Part of the Strategy Software pattern.
     */
    class LogNPlane final : public PlaneEventAlgorithm {
        /**
         * Specifies the arguments to be passed to geoSubsetCallback in findPlane
         */
        using OptimalPlaneLog = OptimalPlane<std::shared_ptr<PlaneEventVector>, const bool>;

    public:
        std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> findPlane(const SplitParam &splitParam) override;

    private:
        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @return The vector of PlaneEvents
        */
        static PlaneEventVector generatePlaneEvents(const SplitParam &splitParam);

        /**
        * When an optimal plane has been found divide the used PlaneEvents for further subdivision through child nodes.
        * @param splitParam Contains information about the current scene to be split.
        * @param planeEvents The events that were generated during {@link findPlane}.
        * @param plane The plane to split the shapes by.
        * @param minSide Whether to include planar shapes to the bounding box closer to the origin.
        * @return The PlaneEventLists for the bounding boxes closer and further away from the origin.
        */
        static PlaneEventVectors<2> generatePlaneEventSubsets(const SplitParam &splitParam, const PlaneEventVector &planeEvents, const Plane &plane, bool minSide);

        /**
         * Classification of where a shape has area relative to a split plane.
         */
        enum class Locale {
            MIN_ONLY,
            MAX_ONLY,
            BOTH
        };

        /**
         * Creates a lookup table for shape indices determining whether the shapes has area only left of, only right of or on both sides of the box divided by the plane.
         * @param events The list of events whose shapes to classify.
         * @param plane The plane tht divides the shapes into two sets.
         * @param minSide
         * @return An unordered_map used for lookups of individual shape locales.
         */
        static std::unordered_map<size_t, Locale> classifyShapesRelativeToPlane(const PlaneEventVector &events, const Plane &plane, bool minSide);

        /**
        * Creates new events for two sub bounding boxes out of shapes that overlap both of them.
        * @param splitParam Contains information about the current scene to be split.
        * @param geoIndices The index list of shapes that straddle the plane.
        * @param plane The plane that splits the scene's bounding box into two new sub boxes.
        * @return Two new PlaneEventLists for the minimal and maximal bounding boxes respectively (unsorted!).
        */
        static std::array<PlaneEventVector, 2> generatePlaneEventsForClippedShapes(const SplitParam &splitParam, const ObjectIndexVector &geoIndices, const Plane &plane);

        /**
         * Takes two sorted PlaneEventLists and merges them in a single merge sort step.
         * @param first The first PlaneEventList.
         * @param second The second PlaneEventList
         * @return A unique_ptr to a combined sorted PlaneEventList.
         */
        static std::unique_ptr<PlaneEventVector> mergePlaneEventLists(const PlaneEventVector &first, const PlaneEventVector &second);
    };
}// namespace kdtree
