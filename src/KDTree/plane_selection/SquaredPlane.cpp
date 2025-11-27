#include "KDTree/plane_selection/SquaredPlane.h"

namespace kdtree {
    // O(N^2) implementation
    std::tuple<Plane, double, std::variant<TriangleIndexVectors<2>, PlaneEventVectors<2>>> SquaredPlane::findPlane(
            const SplitParam &splitParam) {
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            throw std::invalid_argument("SquaredPlane does not support PlaneEventLists in SplitParam argument");
        }
        const auto &boundFaces = std::get<ObjectIndexVector>(splitParam.boundObjects);
        const std::function geoSubsetCallback = [this](const OptimalPlaneSquared &, TriangleIndexVectors<3> indexVectors, const bool minSideChosen) {
            return addEqualPointsToSubset<TriangleIndexVectors>(std::move(indexVectors), minSideChosen);
        };
        OptimalPlaneSquared optPlane{splitParam, geoSubsetCallback};
        //each vertex proposes a split plane candidate: test for each of them, store them in buffer set to avoid duplicate testing
        std::unordered_set<double> testedPlaneCoordinates{};
        auto [vertex_begin, vertex_end] = transformIterator(boundFaces.cbegin(), boundFaces.cend(),
                                                              splitParam.geometryObjects);
        std::mutex testedPlaneMutex{};
        thrust::for_each(thrust::device, vertex_begin, vertex_end,
                         [&splitParam, &optPlane, &testedPlaneCoordinates, &testedPlaneMutex](
                                 const auto &indexAndVertices) {
                             const auto [index, vertices] = indexAndVertices;
                             //first clip the triangles vertices to the current bounding box and then get the bounding box of the clipped triangle -> use the box edges as split plane candidates
                             const auto clippedVertices = splitParam.boundingBox.clipToVoxel(vertices);
                             const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Vertex>>(
                                     clippedVertices);
                             for (const auto planeSurfacePoint: {minPoint, maxPoint}) {
                                 //constructs the plane that goes through a vertex lying on the bounding box of the face to be checked and spans in a specified direction.
                                 Plane candidatePlane{
                                         planeSurfacePoint[static_cast<int>(splitParam.splitDirection)],
                                         splitParam.splitDirection};
                                 {
                                     //continue if plane has already been tested
                                     std::lock_guard lock{testedPlaneMutex};
                                     if (testedPlaneCoordinates.find(candidatePlane.axisCoordinate) !=
                                         testedPlaneCoordinates.cend()) {
                                         continue;
                                     }
                                     testedPlaneCoordinates.emplace(candidatePlane.axisCoordinate);
                                 }

                                 auto triangleIndexLists = containedTriangles(splitParam, candidatePlane);

                                 //evaluate the candidate plane and store if it is better than the currently stored result
                                 auto [candidateCost, minSideChosen] = costForPlane(
                                         splitParam.boundingBox, candidatePlane, triangleIndexLists[0]->size(),
                                         triangleIndexLists[1]->size(), triangleIndexLists[2]->size());
                                 {
                                     // this if clause exists to consistently build the same KDTree (choose plane with lower coordinate) by eliminating indeterministic behavior should the cost be equal.
                                     // this is not important for functionality but for testing purposes
                                     if (candidateCost == optPlane.getCost() && optPlane.getOptimalPlane().axisCoordinate < candidatePlane.axisCoordinate) {
                                         continue;
                                     }
                                     optPlane.evaluatePlane(candidatePlane, candidateCost, std::move(triangleIndexLists), minSideChosen);
                                 }
                             }
                         });
        return std::make_tuple(optPlane.getOptimalPlane(), optPlane.getCost(), optPlane.getPointsSplit());
    }

    TriangleIndexVectors<3> SquaredPlane::containedTriangles(const SplitParam &splitParam, const Plane &split) {
        using namespace kdtree;
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            throw std::invalid_argument("SquaredPlane does not support PlaneEventLists in SplitParam argument");
        }
        const auto &boundFaces = std::get<ObjectIndexVector>(splitParam.boundObjects);
        //define three sets of triangles: closer to the origin, further away, in the plane
        auto index_less = std::make_unique<ObjectIndexVector>();
        auto index_greater = std::make_unique<ObjectIndexVector>();
        auto index_equal = std::make_unique<ObjectIndexVector>();
        index_less->reserve(boundFaces.size() / 2);
        index_greater->reserve(boundFaces.size() / 2);


        //perform check for every triangle contained in this node's bounding box.
        //transform faceIndices into the vertices
        auto [begin, end] = transformIterator(boundFaces.cbegin(), boundFaces.cend(), splitParam.geometryObjects);
        std::for_each(
                begin, end,
                [&splitParam, &split, &index_greater, &index_less, &index_equal](std::pair<size_t, std::vector<Vertex>> pair) {
                    auto [faceIndex, vertices] = pair;
                    bool less{false}, greater{false};
                    auto clippedVertices = splitParam.boundingBox.clipToVoxel(vertices);
                    for (const Vertex vertex: clippedVertices) {
                        //vertex is closer to the origin than the plane
                        if (vertex[static_cast<int>(split.orientation)] < split.axisCoordinate && !less) {
                            less = true;
                            //triangle has area in the closer bounding box and needs to be checked there for intersections
                            index_less->push_back(faceIndex);
                        }
                        //vertex is farther away of the origin than the plane
                        else if (vertex[static_cast<int>(split.orientation)] > split.axisCoordinate && !greater) {
                            greater = true;
                            //triangle has area in the greater bounding box and needs to be checked there for intersections
                            index_greater->push_back(faceIndex);
                        }
                    }
                    //all vertices of the triangle lie in the plane -> triangle lies in the plane
                    if (!less && !greater) {
                        index_equal->push_back(faceIndex);
                    }
                });
        return std::array{std::move(index_less), std::move(index_greater), std::move(index_equal)};
    }
}// namespace kdtree
