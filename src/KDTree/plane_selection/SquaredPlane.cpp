#include "KDTree/plane_selection/SquaredPlane.h"

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
        //each vertex proposes a split plane candidate: test for each of them, store them in a per-thread
        //buffer set to avoid duplicate testing. The set is kept per-thread (rather than shared behind a
        //lock) since duplicate testing across threads is harmless.
        //no structured binding here - clang can't capture those in OMP regions
        const auto vertexRange = transformIterator(boundObjects.cbegin(), boundObjects.cend(),
                                                   splitParam.geometryObjects);
        const auto vertex_begin = vertexRange.first;
        const auto vertex_end = vertexRange.second;
        const std::ptrdiff_t vertexCount = std::distance(vertex_begin, vertex_end);

        //Evaluates both candidate planes proposed by a single vertex against optPlane, deduplicating
        //against testedPlaneCoordinates. Shared between the sequential and parallel paths below so the
        //per-vertex logic only exists once.
        const auto evaluateVertexCandidates = [&splitParam, this](const std::pair<size_t, std::vector<Vertex>> &indexAndVertices,
                                                                  std::unordered_set<double> &testedPlaneCoordinates,
                                                                  OptimalPlaneSquared &optPlane) {
            const auto &vertices = indexAndVertices.second;
            //first clip the shapes vertices to the current bounding box and then get the bounding box of the clipped shape -> use the box edges as split plane candidates
            const auto clippedVertices = splitParam.boundingBox.clipToVoxel(vertices);
            const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Vertex>>(clippedVertices);
            for (const auto planeSurfacePoint: {minPoint, maxPoint}) {
                //constructs the plane that goes through a vertex lying on the bounding box of the shape to be checked and spans in a specified direction.
                Plane candidatePlane{
                        planeSurfacePoint[static_cast<int>(splitParam.splitDirection)],
                        splitParam.splitDirection};
                if (testedPlaneCoordinates.contains(candidatePlane.axisCoordinate)) {
                    continue;
                }
                testedPlaneCoordinates.emplace(candidatePlane.axisCoordinate);

                auto shapeIndexLists = containedShapes(splitParam, candidatePlane);

                //evaluate the candidate plane against the current best so far.
                auto [candidateCost, minSideChosen] = costForPlane(
                        splitParam.boundingBox, candidatePlane, shapeIndexLists[0]->size(),
                        shapeIndexLists[1]->size(), shapeIndexLists[2]->size(), splitParam.particleMode);
                optPlane.evaluatePlane(candidatePlane, candidateCost, std::move(shapeIndexLists), minSideChosen);
            }
        };

        if (vertexCount <= static_cast<std::ptrdiff_t>(constants::SQUARED_PLANE_PARALLEL_THRESHOLD)) {
            //nodes this small are common (most tree nodes are near the leaves, where bounding boxes
            //contain few shapes) and not worth the OMP dispatch overhead - thread-team spin-up plus a
            //heap-allocated OptimalPlaneSquared and unordered_set per thread - which isn't amortized by
            //this little work. Mirrors LeafNode.cpp's LEAF_THRUST_PARALLEL_THRESHOLD guard.
            OptimalPlaneSquared optPlane{splitParam, geoSubsetCallback, /*useLocking=*/false};
            std::unordered_set<double> testedPlaneCoordinates;
            for (auto it = vertex_begin; it != vertex_end; ++it) {
                evaluateVertexCandidates(*it, testedPlaneCoordinates, optPlane);
            }
            return std::make_tuple(optPlane.getOptimalPlane(), optPlane.getCost(), optPlane.getPointsSplit());
        }

        //per-chunk buffers instead of per-thread ones, so it doesn't matter which thread runs which chunk.
        //prebuildTree() calls us from inside its omp parallel/task tree, where a nested parallel for would
        //just get a team of 1 -> hand out tasks to the existing team instead.
        const bool nested = omp_in_parallel();
        const int teamSize = nested ? omp_get_num_threads() : omp_get_max_threads();
        //a few chunks per thread for load balancing, but not more than the work is worth
        const std::ptrdiff_t chunkCount = std::clamp<std::ptrdiff_t>(
                vertexCount / static_cast<std::ptrdiff_t>(constants::SQUARED_PLANE_PARALLEL_THRESHOLD),
                1, static_cast<std::ptrdiff_t>(teamSize) * 4);

        std::vector<std::unique_ptr<OptimalPlaneSquared>> chunkOptPlanes(chunkCount);
        const auto processChunk = [&](const std::ptrdiff_t chunk) {
            const std::ptrdiff_t begin = vertexCount * chunk / chunkCount;
            const std::ptrdiff_t end = vertexCount * (chunk + 1) / chunkCount;
            //no locking needed, each chunk owns its plane
            chunkOptPlanes[chunk] = std::make_unique<OptimalPlaneSquared>(splitParam, geoSubsetCallback, false);
            std::unordered_set<double> testedPlaneCoordinates;
            for (std::ptrdiff_t i = begin; i < end; ++i) {
                evaluateVertexCandidates(*(vertex_begin + i), testedPlaneCoordinates, *chunkOptPlanes[chunk]);
            }
        };

        if (nested) {
#pragma omp taskloop grainsize(1) shared(processChunk)
            for (std::ptrdiff_t chunk = 0; chunk < chunkCount; ++chunk) {
                processChunk(chunk);
            }
        } else {
#pragma omp parallel shared(processChunk)
#pragma omp single
#pragma omp taskloop grainsize(1) shared(processChunk)
            for (std::ptrdiff_t chunk = 0; chunk < chunkCount; ++chunk) {
                processChunk(chunk);
            }
        }
        //sequential merge: at most chunkCount candidates, tie-break keeps it deterministic
        OptimalPlaneSquared *bestOptPlane = chunkOptPlanes[0].get();
        for (std::size_t chunk = 1; chunk < chunkOptPlanes.size(); ++chunk) {
            bestOptPlane = &planeWithLesserCost(*bestOptPlane, *chunkOptPlanes[chunk]);
        }
        //generate the shape index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return std::make_tuple(bestOptPlane->getOptimalPlane(), bestOptPlane->getCost(), bestOptPlane->getPointsSplit());
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
