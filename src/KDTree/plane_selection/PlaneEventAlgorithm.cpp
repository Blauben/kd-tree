#include "KDTree/plane_selection/PlaneEventAlgorithm.h"

namespace kdtree {
    TriangleCounter::TriangleCounter(const size_t dimensionCount, const std::array<size_t, 3> &initialValues)
        : dimensionTriangleValues(dimensionCount, initialValues) {
        if (dimensionCount == 0) {
            throw std::invalid_argument("Dimension count must be greater than zero");
        }
    }

    void TriangleCounter::updateMax(Direction direction, const size_t p_planar, const size_t p_end) {
        dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(1) -= p_planar +
                                                                                                             p_end;
    }

    void TriangleCounter::updateMin(Direction direction, const size_t p_planar, const size_t p_start) {
        dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(0) += p_planar +
                                                                                                             p_start;
    }

    void TriangleCounter::setPlanar(Direction direction, const size_t p_planar) {
        dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(2) = p_planar;
    }

    size_t TriangleCounter::getMin(Direction direction) const {
        return dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(0);
    }

    size_t TriangleCounter::getMax(Direction direction) const {
        return dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(1);
    }

    size_t TriangleCounter::getPlanar(Direction direction) const {
        return dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(2);
    }

    PlaneEventVector PlaneEventAlgorithm::generatePlaneEventsFromFaces(const SplitParam &splitParam,
                                                                       std::vector<Direction> directions) {
        // each face has min and max point and each proposes a plane in each of the directions
        PlaneEventVector events{};
        events.reserve(countFaces(splitParam.boundObjects) * 2 * directions.size());
        //mutex used for synchronizing insertions through threads
        std::mutex eventsMutex{};
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            return std::get<PlaneEventVector>(splitParam.boundObjects);
        }
        const auto &boundTriangles{std::get<ObjectIndexVector>(splitParam.boundObjects)};
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = transformIterator(boundTriangles.cbegin(), boundTriangles.cend(),
                                                              splitParam.vertices, splitParam.faces);
        thrust::for_each(thrust::device, vertex3_begin, vertex3_end,
                         [&splitParam, &events, &directions, &eventsMutex](const auto &indexAndTriplet) {
                             const auto [index, triplet] = indexAndTriplet;
                             //first clip the triangles vertices to the current bounding box and then get the bounding box of the clipped triangle -> use the box edges as split plane candidates
                             const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Array3>>(
                                     splitParam.boundingBox.clipToVoxel(triplet));
                             std::lock_guard lock(eventsMutex);
                             for (const auto &direction: directions) {
                                 // if the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
                                 if (minPoint[static_cast<int>(direction)] == maxPoint[static_cast<int>(direction)]) {
                                     events.emplace_back(
                                             PlaneEventType::planar,
                                             Plane(minPoint, direction),
                                             index);
                                     return;
                                 }
                                 //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
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
