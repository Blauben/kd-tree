#include "KDTree/plane_selection/SquaredPlane.h"

#include <utility>

namespace kdtree {
    // O(N^2) implementation
    std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> SquaredPlane::findPlane(
            const SplitParam &splitParam) {
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            throw std::invalid_argument("SquaredPlane does not support PlaneEventLists in SplitParam argument");
        }
        const auto &boundObjects = std::get<ObjectIndexVector>(splitParam.boundObjects);
        //split the geometry through the optimal plane
        const std::function geoSubsetCallback = [this](const OptimalPlaneSquared &, ObjectIndexVectors<3> indexVectors, const bool minSideChosen) {
            return addEqualPointsToSubset<ObjectIndexVectors>(std::move(indexVectors), minSideChosen);
        };
        //init the OptPlane to evaluate generated planes
        OptimalPlaneSquared optPlane{splitParam, geoSubsetCallback};
        //each vertex proposes a split plane candidate: test for each of them, store them in buffer set to avoid duplicate testing
        std::unordered_set<double> testedPlaneCoordinates{};
        auto [vertex_begin, vertex_end] = transformIterator(boundObjects.cbegin(), boundObjects.cend(),
                                                            splitParam.geometryObjects);
        thrust::for_each(thrust::host, vertex_begin, vertex_end,
                         [&splitParam, &optPlane, &testedPlaneCoordinates](
                                 const auto &indexAndVertices) {
                             const auto [index, vertices] = indexAndVertices;
                             //first clip the shapes vertices to the current bounding box and then get the bounding box of the clipped shape -> use the box edges as split plane candidates
                             const auto clippedVertices = splitParam.boundingBox.clipToVoxel(vertices);
                             const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Vertex>>(clippedVertices);
                             for (const auto planeSurfacePoint: {minPoint, maxPoint}) {
                                 //constructs the plane that goes through a vertex lying on the bounding box of the shape to be checked and spans in a specified direction.
                                 Plane candidatePlane{
                                         planeSurfacePoint[static_cast<int>(splitParam.splitDirection)],
                                         splitParam.splitDirection};
                                 {
                                     if (testedPlaneCoordinates.contains(candidatePlane.axisCoordinate)) {
                                         continue;
                                     }
                                     testedPlaneCoordinates.emplace(candidatePlane.axisCoordinate);
                                 }

                                 auto shapeIndexLists = containedShapes(splitParam, candidatePlane);

                                 //evaluate the candidate plane and store if it is better than the currently stored result
                                 auto [candidateCost, minSideChosen] = costForPlane(
                                         splitParam.boundingBox, candidatePlane, shapeIndexLists[0]->size(),
                                         shapeIndexLists[1]->size(), shapeIndexLists[2]->size(), splitParam.particleMode);
                                 {
                                     // this if-clause exists to consistently build the same KDTree (choose plane with lower coordinate) by eliminating indeterministic behavior should the cost be equal.
                                     // this is not important for functionality but for testing purposes
                                     if (candidateCost == optPlane.getCost() && optPlane.getOptimalPlane().axisCoordinate < candidatePlane.axisCoordinate) {
                                         continue;
                                     }
                                     optPlane.evaluatePlane(candidatePlane, candidateCost, std::move(shapeIndexLists), minSideChosen);
                                 }
                             }
                         });
        //generate the shape index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return std::make_tuple(optPlane.getOptimalPlane(), optPlane.getCost(), optPlane.getPointsSplit());
    }

    ObjectIndexVectors<3> SquaredPlane::containedShapes(const SplitParam &splitParam, const Plane &split) {
        using namespace kdtree;
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            throw std::invalid_argument("SquaredPlane does not support PlaneEventLists in SplitParam argument");
        }
        const auto &boundGeometry = std::get<ObjectIndexVector>(splitParam.boundObjects);
        //define three sets of shapes: closer to the origin, further away, in the plane
        auto index_less = std::make_unique<ObjectIndexVector>();
        auto index_greater = std::make_unique<ObjectIndexVector>();
        auto index_equal = std::make_unique<ObjectIndexVector>();
        index_less->reserve(boundGeometry.size() / 2);
        index_greater->reserve(boundGeometry.size() / 2);


        //perform check for every shape contained in this node's bounding box.
        //transform shapeIndices into the vertices
        auto [begin, end] = transformIterator(boundGeometry.cbegin(), boundGeometry.cend(), splitParam.geometryObjects);
        std::for_each(
                begin, end,
                [&splitParam, &split, &index_greater, &index_less, &index_equal](std::pair<size_t, std::vector<Vertex>> pair) {
                    auto [geoIndex, vertices] = std::move(pair);
                    bool less{false}, greater{false};
                    auto clippedVertices = splitParam.boundingBox.clipToVoxel(vertices);
                    for (const auto &vertex: clippedVertices) {
                        //vertex is closer to the origin than the plane
                        if (vertex[static_cast<int>(split.orientation)] < split.axisCoordinate && !less) {
                            less = true;
                            //shape has area in the closer bounding box and needs to be checked there for intersections
                            index_less->push_back(geoIndex);
                        }
                        //vertex is farther away of the origin than the plane
                        else if (vertex[static_cast<int>(split.orientation)] > split.axisCoordinate && !greater) {
                            greater = true;
                            //shape has area in the greater bounding box and needs to be checked there for intersections
                            index_greater->push_back(geoIndex);
                        }
                    }
                    //all vertices of the shape lie in the plane -> shape lies in the plane
                    if (!less && !greater) {
                        index_equal->push_back(geoIndex);
                    }
                });
        return std::array{std::move(index_less), std::move(index_greater), std::move(index_equal)};
    }
}// namespace kdtree
