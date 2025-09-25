#include "KDTree/plane_selection/SquaredPlane.h"

namespace kdtree {
    // O(N^2) implementation
    std::tuple<Plane, double, PointIndexVectors<2>> SquaredPlane::findPlane(
        const SplitParam &splitParam) {
        //initialize the default plane and make it costly
        double cost = std::numeric_limits<double>::infinity();
        Plane optPlane{0, splitParam.splitDirection};
        //store the pointSets that are implicitly generated during plane testing for later use.
        PointIndexVectors<2> optPointIndexLists{};
        //each point proposes a split plane candidate: test for each of them, store them in buffer set to avoid duplicate testing
        std::unordered_set<double> testedPlaneCoordinates{};
        std::mutex optMutex{}, testedPlaneMutex{};
        thrust::for_each(thrust::device, splitParam.points.cbegin(), splitParam.points.cend(),
                         [&splitParam, &optPlane, &cost, &optPointIndexLists, &testedPlaneCoordinates, &optMutex, &
                             testedPlaneMutex](
                     const auto &point) {
                             //first clip the triangles vertices to the current bounding box and then get the bounding box of the clipped triangle -> use the box edges as split plane candidates
                             Plane candidatePlane{
                                     point[static_cast<int>(splitParam.splitDirection)],
                                     splitParam.splitDirection
                                 }; {
                                     //continue if plane has already been tested
                                     std::lock_guard lock{testedPlaneMutex};
                                     if (testedPlaneCoordinates.find(candidatePlane.axisCoordinate) !=
                                         testedPlaneCoordinates.cend()) {
                                         return;
                                     }
                                     testedPlaneCoordinates.emplace(candidatePlane.axisCoordinate);
                                 }

                                 auto triangleIndexLists = containedPoints(splitParam, candidatePlane);

                                 //evaluate the candidate plane and store if it is better than the currently stored result
                                 auto [candidateCost, minSideChosen] = costForPlane(
                                     splitParam.boundingBox, candidatePlane, triangleIndexLists[0]->size(),
                                     triangleIndexLists[1]->size(), triangleIndexLists[2]->size()); {
                                     std::lock_guard lock(optMutex);
                                     // this if clause exists to consistently build the same KDTree (choose plane with lower coordinate) by eliminating indeterministic behavior should the cost be equal.
                                     // this is not important for functionality but for testing purposes
                                     if (candidateCost == cost && optPlane.axisCoordinate < candidatePlane.
                                         axisCoordinate) {
                                         return;
                                     }
                                     if (candidateCost <= cost) {
                                         cost = candidateCost;
                                         optPlane = candidatePlane;
                                         //planar faces have to be included in one of the two sub boxes.
                                         const auto &includePlanarTo = triangleIndexLists[minSideChosen ? 0 : 1];
                                         includePlanarTo->insert(includePlanarTo->cend(),
                                                                 triangleIndexLists[2]->cbegin(),
                                                                 triangleIndexLists[2]->cend());
                                         optPointIndexLists = {
                                             std::move(triangleIndexLists[0]), std::move(triangleIndexLists[1])
                                         };
                                     }
                                 }
                         });
        return std::make_tuple(optPlane, cost, std::move(optPointIndexLists));
    }

    PointIndexVectors<3> SquaredPlane::containedPoints(const SplitParam &splitParam, const Plane &split) {
        using namespace kdtree;
        //define three sets of triangles: closer to the origin, further away, in the plane
        auto index_less = std::make_unique<PointIndexVector>();
        auto index_greater = std::make_unique<PointIndexVector>();
        auto index_equal = std::make_unique<PointIndexVector>();
        index_less->reserve(splitParam.boundPoints.size() / 2);
        index_greater->reserve(splitParam.boundPoints.size() / 2);


        std::for_each(splitParam.boundPoints.cbegin(), splitParam.boundPoints.cend(),
            [&splitParam, &split, &index_greater, &index_less, &index_equal](const unsigned pointIndex) {
                const auto point = splitParam.points[pointIndex];
                    if (point[static_cast<int>(split.orientation)] < split.axisCoordinate) {
                        //triangle has area in the closer bounding box and needs to be checked there for intersections
                        index_less->push_back(pointIndex);
                    }
                    //point is farther away of the origin than the plane
                    else if (point[static_cast<int>(split.orientation)] > split.axisCoordinate) {
                        //triangle has area in the greater bounding box and needs to be checked there for intersections
                        index_greater->push_back(pointIndex);
                    } else {
                        index_equal->push_back(pointIndex);
                    }
            });
        return std::array{std::move(index_less), std::move(index_greater), std::move(index_equal)};
    }
} // namespace kdtree
