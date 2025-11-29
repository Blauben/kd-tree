#include "KDTree/plane_selection/PlaneEventAlgorithm.h"

namespace kdtree {
    ShapeCounter::ShapeCounter(const size_t dimensionCount, const std::array<size_t, 3> &initialValues)
        : dimensionShapeValues(dimensionCount, initialValues) {
        if (dimensionCount == 0) {
            throw std::invalid_argument("Dimension count must be greater than zero");
        }
    }

    void ShapeCounter::updateMax(Direction direction, const size_t p_planar, const size_t p_end) {
        dimensionShapeValues.at(static_cast<size_t>(direction) % dimensionShapeValues.size()).at(1) -= p_planar +
                                                                                                       p_end;
    }

    void ShapeCounter::updateMin(Direction direction, const size_t p_planar, const size_t p_start) {
        dimensionShapeValues.at(static_cast<size_t>(direction) % dimensionShapeValues.size()).at(0) += p_planar +
                                                                                                       p_start;
    }

    void ShapeCounter::setPlanar(Direction direction, const size_t p_planar) {
        dimensionShapeValues.at(static_cast<size_t>(direction) % dimensionShapeValues.size()).at(2) = p_planar;
    }

    size_t ShapeCounter::getMin(Direction direction) const {
        return dimensionShapeValues.at(static_cast<size_t>(direction) % dimensionShapeValues.size()).at(0);
    }

    size_t ShapeCounter::getMax(Direction direction) const {
        return dimensionShapeValues.at(static_cast<size_t>(direction) % dimensionShapeValues.size()).at(1);
    }

    size_t ShapeCounter::getPlanar(Direction direction) const {
        return dimensionShapeValues.at(static_cast<size_t>(direction) % dimensionShapeValues.size()).at(2);
    }

    PlaneEventVector PlaneEventAlgorithm::generatePlaneEventsFromGeometry(const SplitParam &splitParam,
                                                                          std::vector<Direction> directions) {
        // each shape has min and max point and each proposes a plane in each of the directions
        PlaneEventVector events{};
        events.reserve(countGeometryObjects(splitParam.boundObjects) * 2 * directions.size());
        //mutex used for synchronizing insertions through threads
        std::mutex eventsMutex{};
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            return std::get<PlaneEventVector>(splitParam.boundObjects);
        }
        const auto &boundShapes{std::get<ObjectIndexVector>(splitParam.boundObjects)};
        //transform the shapes into vertices
        auto [vertex_begin, vertex_end] = transformIterator(boundShapes.cbegin(), boundShapes.cend(),
                                                            splitParam.geometryObjects);
        thrust::for_each(thrust::device, vertex_begin, vertex_end,
                         [&splitParam, &events, &directions, &eventsMutex](const auto &indexAndVertices) {
                             const auto [index, vertices] = indexAndVertices;
                             //first clip the shapes vertices to the current bounding box and then get the bounding box of the clipped shape -> use the box edges as split plane candidates
                             const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Vertex>>(
                                     splitParam.boundingBox.clipToVoxel(vertices));
                             std::lock_guard lock(eventsMutex);
                             for (const auto &direction: directions) {
                                 // if the shape is perpendicular to the split direction, generate a planar event with the candidate plane in which the shape lies
                                 if (minPoint[static_cast<int>(direction)] == maxPoint[static_cast<int>(direction)]) {
                                     events.emplace_back(
                                             PlaneEventType::planar,
                                             Plane(minPoint, direction),
                                             index);
                                     return;
                                 }
                                 //else create a starting and ending event consisting of the planes defined by the min and max points of the shape's bounding box.
                                 events.emplace_back(
                                         PlaneEventType::starting,
                                         Plane(minPoint, direction),
                                         index);
                                 events.emplace_back(
                                         PlaneEventType::ending,
                                         Plane(maxPoint, direction),
                                         index);
                             }
                         });
        //reduce size
        events.shrink_to_fit();
        //sort the events by plane position and then by PlaneEventType. Refer to {@link PlaneEventType} for the specific order
        std::sort(events.begin(), events.end());
        return events;
    }

}// namespace kdtree
