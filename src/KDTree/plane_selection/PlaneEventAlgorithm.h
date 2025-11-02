#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <variant>
#include <vector>

#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/util/UtilityContainer.h"
#include "thrust/detail/execution_policy.h"
#include "thrust/execution_policy.h"
#include "thrust/iterator/iterator_facade.h"
#include "thrust/iterator/transform_iterator.h"
#include "thrust/system/detail/sequential/for_each.h"

namespace kdtree {
struct SplitParam;

    /**
     * Helper class to keep count of triangles that straddle split planes, while PlaneEvents are iterated over.
     */
    class TriangleCounter {
    public:
        /**
         * Update the triangle count for the max side of the plane.
         * @param direction For which dimension to update values.
         * @param p_planar Amount of faces lying in the plane.
         * @param p_end Amount of faces ending in the plane.
         */
        void updateMax(Direction direction, size_t p_planar, size_t p_end);

        /**
        * Update the triangle count for the min side of the plane.
        * @param direction For which dimension to update values.
        * @param p_planar Amount of faces lying in the plane.
        * @param p_start Amount of faces starting in the plane.
        */
        void updateMin(Direction direction, size_t p_planar, size_t p_start);

        /**
         * Sets the amount of planar faces for a specific dimension.
         * @param direction For which dimension to update values.
         * @param p_planar Amount of faces lying in the plane.
         */
        void setPlanar(Direction direction, size_t p_planar);

        /**
         * Returns min value for a specific dimension.
         * @param direction The dimension.
         * @return The min value.
         */
        [[nodiscard]] size_t getMin(Direction direction) const;

        /**
         * Returns max value for a specific dimension.
         * @param direction The dimension.
         * @return The max value.
         */
        [[nodiscard]] size_t getMax(Direction direction) const;

        /**
         * Returns the amount of planar faces for a specific dimension.
         * @param direction The dimension.
         * @return The planar face amount.
         */
        [[nodiscard]] size_t getPlanar(Direction direction) const;

        TriangleCounter(size_t dimensionCount, const std::array<size_t, 3> &initialValues);

    private:
        /**
         * Keeps track of triangle counts for every dimension in order MIN, MAX, PLANAR
         */
        std::vector<std::array<size_t, 3> > dimensionTriangleValues;
    };

    class PlaneEventAlgorithm : public PlaneSelectionAlgorithm {
    protected:
        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes using an index list of faces. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @param directions For which directions to generate the events for.
        * @return The vector of PlaneEvents.
        */
        static PlaneEventVector generatePlaneEventsFromFaces(const SplitParam &splitParam,
                                                             std::vector<Direction> directions);

        /**
         * Iterates over PlaneEvents and determines the optimal split plane.
         * @param events The events to base calculations on.
         * @param triangleCounter Used to track face count during iteration.
         * @param boundingBox The current node's bounding box
         * @return Tuple of optimal plane, its cost and where to include planar faces.
         */
        template <typename... OptimalPlaneArgs>
        static void traversePlaneEvents(OptimalPlane<OptimalPlaneArgs...>& optPlane, const PlaneEventVector &events,
                                                                   TriangleCounter &triangleCounter) {
            const Box& boundingBox = optPlane.splitParam.boundingBox;
        //traverse all the events
        int i{0};
        while (i < events.size()) {
            //poll a plane to test
            const Plane &candidatePlane = events[i].plane;
            //for each plane calculate the faces whose vertices lie in the plane. Differentiate between the face starting in the plane, ending in the plane or all vertices lying in the plane
            size_t p_start{0}, p_end{0}, p_planar{0};
            //count all faces that end in the plane, this works because the PlaneEvents are sorted by position and then by PlaneEventType
            while (i < events.size() && events[i].plane.orientation == candidatePlane.orientation && events[i].plane.
                   axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::ending) {
                p_end++;
                i++;
            }
            //count all the faces that lie in the plane
            while (i < events.size() && events[i].plane.orientation == candidatePlane.orientation && events[i].plane.
                   axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::planar) {
                p_planar++;
                i++;
            }
            //count all the faces that start in the plane
            while (i < events.size() && events[i].plane.orientation == candidatePlane.orientation && events[i].plane.
                   axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::starting) {
                p_start++;
                i++;
            }
            //reference to the triangle counter of the current dimension -> better readability
            triangleCounter.setPlanar(candidatePlane.orientation, p_planar);
            triangleCounter.updateMax(candidatePlane.orientation, p_planar, p_end);
            //evaluate plane and update should the new plane be more efficient
            auto [candidateCost, minSideChosen] = costForPlane(boundingBox, candidatePlane,
                                                               triangleCounter.getMin(candidatePlane.orientation),
                                                               triangleCounter.getMax(candidatePlane.orientation),
                                                               triangleCounter.getPlanar(candidatePlane.orientation));
            // this condition exists to consistently build the same KDTree (choose plane with lower coordinate) by eliminating indeterministic behavior should the cost be equal.
            // this is not important for functionality but for testing purposes
            bool skipEvaluation = candidateCost == optPlane.getCost() && optPlane.getOptimalPlane().axisCoordinate < candidatePlane.axisCoordinate;
            if (!skipEvaluation) {
                optPlane.evaluatePlane(candidatePlane, candidateCost, events, minSideChosen);
            }
            //shift the plane to the next candidate and prepare next iteration
            triangleCounter.updateMin(candidatePlane.orientation, p_planar, p_start);
            triangleCounter.setPlanar(candidatePlane.orientation, 0);
        }
    }
    };
} // namespace kdtree
