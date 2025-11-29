#include "KDTree/plane_selection/LogNSquaredPlane.h"

namespace kdtree {
    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>>
    LogNSquaredPlane::findPlane(const SplitParam &splitParam) {
        //split the geometry through the optimal plane
        const std::function geoSubsetCallback = [](const OptimalPlaneLogNSquared &optPlane, const std::shared_ptr<PlaneEventVector> &events, const bool minSideChosen) {
            return generateGeometrySubsets(events, optPlane.getOptimalPlane(), minSideChosen);
        };
        //init the OptPlane to evaluate generated planes
        OptimalPlaneLogNSquared optPlane{splitParam, geoSubsetCallback};
        for (const auto dimension: ALL_DIRECTIONS) {
            splitParam.splitDirection = dimension;
            findPlaneForSingleDimension(optPlane);
            // this if clause exists to consistently build the same KDTree (choose plane with lower coordinate) by eliminating indeterministic behavior should the cost be equal.
            // this is not important for functionality but for testing purposes
        }
        //generate the shape index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return {optPlane.getOptimalPlane(), optPlane.getCost(), optPlane.getPointsSplit()};
    }

    void LogNSquaredPlane::findPlaneForSingleDimension(OptimalPlaneLogNSquared &optPlane) {
        const auto events = std::make_shared<PlaneEventVector>(generatePlaneEventsFromGeometry(optPlane.splitParam, {optPlane.splitParam.splitDirection}));
        ShapeCounter shapeCounter{1, {0, countGeometryObjects(optPlane.splitParam.boundObjects), 0}};
        traversePlaneEvents(optPlane, events, shapeCounter);
    }

    ObjectIndexVectors<2> LogNSquaredPlane::generateGeometrySubsets(const std::shared_ptr<PlaneEventVector> &planeEvents,
                                                                    const Plane &plane, const bool minSide) {
        auto geometryMin = std::make_unique<ObjectIndexVector>();
        auto geometryMax = std::make_unique<ObjectIndexVector>();
        //set data structure to avoid processing shapes twice -> introduces O(1) lookup instead of O(n) lookup using the vectors directly
        std::unordered_set<size_t> geometryMinLookup{};
        std::unordered_set<size_t> geometryMaxLookup{};
        //each shape will most of the time generate two events, the split plane will try to distribute the shapes evenly
        //Thus reserving 0.5 * 0.5 * planeEvents.size() for each vector
        geometryMin->reserve(planeEvents->size() / 4);
        geometryMax->reserve(planeEvents->size() / 4);
        geometryMinLookup.reserve(planeEvents->size() / 4);
        geometryMaxLookup.reserve(planeEvents->size() / 4);
        std::array<std::mutex, 2> geometryMutex{};
        thrust::for_each(thrust::device, planeEvents->cbegin(), planeEvents->cend(),
                         [&geometryMin, &geometryMax, &plane, minSide, &geometryMinLookup, &geometryMaxLookup, &geometryMutex](
                                 const auto &event) {
                             //lambda function to combine lookup and insertion into one place
                             auto insertIfAbsent = [&geometryMin, &geometryMinLookup, &geometryMax, &geometryMaxLookup, &geometryMutex](const size_t geoIndex, const uint8_t index) {
                                 const auto &vector = index == 0 ? geometryMin : geometryMax;
                                 auto &lookup = index == 1 ? geometryMinLookup : geometryMaxLookup;
                                 std::lock_guard lock(geometryMutex[index]);
                                 if (!lookup.contains(geoIndex)) {
                                     lookup.insert(geoIndex);
                                     vector->push_back(geoIndex);
                                 }
                                 // Since each shape can only be referenced by max two events (since there are only two planes encasing it),
                                 // after the shape has been already been processed once, it can be removed from the lookup buffer after the second time to save space
                                 else {
                                     lookup.erase(lookup.find(geoIndex));
                                 }
                             };
                             //sort the shapes by inferring their position from the event's candidate split plane
                             if (event.plane.axisCoordinate != plane.axisCoordinate) {
                                 insertIfAbsent(event.objIndex,
                                                event.plane.axisCoordinate < plane.axisCoordinate ? 0 : 1);
                             }
                             //the shape is in, starting or ending in the plane to split by -> the PlanarEventType signals its position then
                             else if (event.type == PlaneEventType::planar) {
                                 //minSide specifies where to include planar shapes
                                 insertIfAbsent(event.objIndex, minSide ? 0 : 1);
                             }
                             //the shape starts in the plane, thus its area overlaps with the bounding box further away from the origin.
                             else if (event.type == PlaneEventType::starting) {
                                 insertIfAbsent(event.objIndex, 1);
                             }
                             //the shaepe ends in the plane, thus its area overlaps with the bounding box closer to the origin.
                             else {
                                 insertIfAbsent(event.objIndex, 0);
                             }
                         });
        return {std::move(geometryMin), std::move(geometryMax)};
    }
}// namespace kdtree
