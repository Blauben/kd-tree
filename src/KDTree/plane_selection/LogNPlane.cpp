#include "KDTree/plane_selection/LogNPlane.h"

namespace kdtree {
    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> LogNPlane::findPlane(
            const SplitParam &splitParam) {
        //split the geometry through the optimal plane
        const std::function geoSubsetCallback = [](const OptimalPlaneLog &optPlane, const std::shared_ptr<PlaneEventVector> &events, const bool minSideChosen) {
            return generatePlaneEventSubsets(optPlane.splitParam, *events, optPlane.getOptimalPlane(), minSideChosen);
        };
        //init the OptPlane to evaluate generated planes
        OptimalPlaneLog optPlane{splitParam, geoSubsetCallback};
        //generate the PlaneEvents or fetch them if already generated
        const auto events = std::make_shared<PlaneEventVector>(generatePlaneEvents(splitParam));
        ShapeCounter shapeCounter{3, {0, countGeometryObjects(splitParam.boundObjects), 0}};
        traversePlaneEvents(optPlane, events, shapeCounter);
        //generate the shape index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return {optPlane.getOptimalPlane(), optPlane.getCost(), optPlane.getPointsSplit()};
    }

    PlaneEventVector LogNPlane::generatePlaneEvents(const SplitParam &splitParam) {
        if (std::holds_alternative<ObjectIndexVector>(splitParam.boundObjects)) {
            return generatePlaneEventsFromGeometry(splitParam, ALL_DIRECTIONS);
        }
        return std::get<PlaneEventVector>(splitParam.boundObjects);
    }

    PlaneEventVectors<2> LogNPlane::generatePlaneEventSubsets(const SplitParam &splitParam,
                                                              const PlaneEventVector &planeEvents, const Plane &plane,
                                                              const bool minSide) {
        const auto geoClassification{classifyShapesRelativeToPlane(planeEvents, plane, minSide)};
        PlaneEventVector planeEventsMin{};
        PlaneEventVector planeEventsMax{};
        ObjectIndexVector geoIndexBoth{};
        planeEventsMin.reserve(planeEvents.size() / 2);
        planeEventsMax.reserve(planeEvents.size() / 2);
        //value estimation taken from source paper
        geoIndexBoth.reserve(std::ceil(std::sqrt(planeEvents.size())));
        std::unordered_set<size_t> processedIndices{};

        auto insertToBothIfAbsent = [&geoIndexBoth, &processedIndices](const auto geoIndex) {
            if (!processedIndices.contains(geoIndex)) {
                processedIndices.insert(geoIndex);
                geoIndexBoth.push_back(geoIndex);
            }
        };

        std::ranges::for_each(planeEvents,
                              [&geoClassification, &planeEventsMin, &planeEventsMax, &insertToBothIfAbsent](const auto &event) {
                                  switch (geoClassification.at(event.objIndex)) {
                                      //shape of event only contributes to min side event can be added to side without clipping because no overlap with split plane
                                      case Locale::MIN_ONLY:
                                          planeEventsMin.push_back(event);
                                          break;
                                      //shape of event only contributes to max side event can be added to side without clipping because no overlap with split plane
                                      case Locale::MAX_ONLY:
                                          planeEventsMax.push_back(event);
                                          break;
                                      //shape has area on both sides -> event has to be discarded and scheduled for separate event generation
                                      case Locale::BOTH:
                                      default:
                                          insertToBothIfAbsent(event.objIndex);
                                  }
                              });

        //generate new plane events for straddling shapes that were discarded previously
        auto [newMinEvents, newMaxEvents] = generatePlaneEventsForClippedShapes(splitParam, geoIndexBoth, plane);
        //merge the new events into the existing sorted lists and return
        return {
                mergePlaneEventLists(planeEventsMin, newMinEvents),
                mergePlaneEventLists(planeEventsMax, newMaxEvents)};
    }

    std::unordered_map<size_t, LogNPlane::Locale> LogNPlane::classifyShapesRelativeToPlane(
            const PlaneEventVector &events, const Plane &plane, const bool minSide) {
        std::unordered_map<size_t, Locale> result{};
        //each shape generates 6 plane events on average, thus the amount of shapes can be roughly estimated.
        result.reserve(events.size() / 6);
        //preparing the map by initializing all shapes with them having area in both sub bounding boxes
        std::ranges::for_each(events, [&result](const auto &event) {
            result[event.objIndex] = Locale::BOTH;
        });
        //now search for conditions proving that the shapes DO NOT have area in both boxes
        std::ranges::for_each(events, [minSide, &result, &plane](const auto &event) {
            if (event.type == PlaneEventType::ending && event.plane.orientation == plane.orientation && event.plane.axisCoordinate <= plane.axisCoordinate) {
                result[event.objIndex] = Locale::MIN_ONLY;
            } else if (event.type == PlaneEventType::starting && event.plane.orientation == plane.orientation && event.plane.axisCoordinate >= plane.axisCoordinate) {
                result[event.objIndex] = Locale::MAX_ONLY;
            } else if (event.type == PlaneEventType::planar && event.plane.orientation == plane.orientation) {
                if (event.plane.axisCoordinate < plane.axisCoordinate || (event.plane.axisCoordinate == plane.axisCoordinate && minSide)) {
                    result[event.objIndex] = Locale::MIN_ONLY;
                }
                if (event.plane.axisCoordinate > plane.axisCoordinate || (event.plane.axisCoordinate == plane.axisCoordinate && !minSide)) {
                    result[event.objIndex] = Locale::MAX_ONLY;
                }
            }
        });
        return result;
    }

    std::array<PlaneEventVector, 2> LogNPlane::generatePlaneEventsForClippedShapes(
            const SplitParam &splitParam, const ObjectIndexVector &geoIndices, const Plane &plane) {
        auto [minBox, maxBox] = splitParam.boundingBox.splitBox(plane);
        PlaneEventVector minEvents{};
        PlaneEventVector maxEvents{};
        //each shape generates six new PlaneEvents and each shape has area in both boxes
        minEvents.resize(geoIndices.size() * 6);
        maxEvents.resize(geoIndices.size() * 6);

        //lambda for creating PlaneEvents from a vertex triplet (shape) in one of the two sub boxes
        const auto createPlaneEvents = [](const auto &vertices, const auto &boundingBox, const size_t geoIndex,
                                          auto destIt) {
            //clip to the voxel
            auto clipped = boundingBox.clipToVoxel(vertices);
            //create split plane anchor points using the bounding box
            const auto [minPoint, maxPoint] = Box::getBoundingBox(clipped);
            //associate parameters for PlaneEvent creation
            std::array<std::pair<const Vertex, PlaneEventType>, 2> planeEventParam{
                    std::make_pair(minPoint, PlaneEventType::starting),
                    std::make_pair(maxPoint, PlaneEventType::ending)};
            //create planes in each dimension, be careful to cluster similar anchor points together.
            size_t planeIndex = 0;
            for (const auto &[point, eventType]: planeEventParam) {
                for (const auto &direction: ALL_DIRECTIONS) {
                    //insert directly for parallelization
                    *(destIt + planeIndex++) = PlaneEvent(eventType, Plane(point, direction), geoIndex);
                }
            }
        };

        //transform shapes to vertices
        auto [begin_it, end_it] = transformIterator(geoIndices.cbegin(), geoIndices.cend(), splitParam.geometryObjects);
        const auto n = std::distance(begin_it, end_it);
        auto pos_begin = thrust::make_zip_iterator(
                thrust::make_tuple(begin_it, thrust::counting_iterator<std::size_t>(0)));
        auto pos_end = thrust::make_zip_iterator(
                thrust::make_tuple(end_it, thrust::counting_iterator<std::size_t>(n)));

        //create new events for each shape in both sub boxes
        thrust::for_each(thrust::device, pos_begin, pos_end,
                         [&minBox, maxBox, &minEvents, &maxEvents, &createPlaneEvents](
                                 const auto &zipped) {
                             const auto &indexAndTriplet = thrust::get<0>(zipped);
                             const auto pos = thrust::get<1>(zipped);

                             const auto &[index, vertexTriplet] = indexAndTriplet;
                             //reserve slots of 6 for the threads using the atomic counters. Size fits because of earlier resize
                             createPlaneEvents(vertexTriplet, minBox, index, minEvents.begin() + pos * 6);
                             createPlaneEvents(vertexTriplet, maxBox, index, maxEvents.begin() + pos * 6);
                         });

        //sort the lists for later merge sort integration
        std::sort(minEvents.begin(), minEvents.end());
        std::sort(maxEvents.begin(), maxEvents.end());

        return {minEvents, maxEvents};
    }

    std::unique_ptr<PlaneEventVector> LogNPlane::mergePlaneEventLists(const PlaneEventVector &first,
                                                                      const PlaneEventVector &second) {
        //mergeSort implementation
        auto first_it{first.cbegin()};
        auto second_it{second.cbegin()};
        auto result{std::make_unique<PlaneEventVector>()};
        result->reserve(first.size() + second.size());

        while (first_it != first.cend() || second_it != second.cend()) {
            if (first_it == first.cend() || second_it == second.cend()) {
                result->push_back(first_it == first.cend() ? *second_it++ : *first_it++);
            } else if (*first_it < *second_it) {
                result->push_back(*first_it++);
            } else {
                result->push_back(*second_it++);
            }
        }
        return result;
    }
}// namespace kdtree
