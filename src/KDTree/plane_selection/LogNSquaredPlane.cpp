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

#if defined(KD_TREE_OMP) or defined(KD_USE_CPP)
    ObjectIndexVectors<2> LogNSquaredPlane::generateGeometrySubsets(
            const std::shared_ptr<PlaneEventVector> &planeEvents,
            const Plane &plane, const bool minSide) {

        const int numThreads = omp_get_max_threads();// set once, globally, at startup

        // Per-thread local buffers — no locking during the parallel phase
        std::vector<ObjectIndexVector> localMin(numThreads), localMax(numThreads);
        std::vector<std::unordered_set<size_t>> localMinLookup(numThreads), localMaxLookup(numThreads);

        for (int t = 0; t < numThreads; ++t) {
            localMin[t].reserve(planeEvents->size() / 4 / numThreads);
            localMax[t].reserve(planeEvents->size() / 4 / numThreads);
            localMinLookup[t].reserve(planeEvents->size() / 4 / numThreads);
            localMaxLookup[t].reserve(planeEvents->size() / 4 / numThreads);
        }

    #pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto &myMin = localMin[tid];
            auto &myMax = localMax[tid];
            auto &myMinLookup = localMinLookup[tid];
            auto &myMaxLookup = localMaxLookup[tid];

            auto insertIfAbsent = [&](const size_t geoIndex, const uint8_t index) {
                auto &vector = index == 0 ? myMin : myMax;
                auto &lookup = index == 0 ? myMinLookup : myMaxLookup;
                if (!lookup.contains(geoIndex)) {
                    lookup.insert(geoIndex);
                    vector.push_back(geoIndex);
                } else {
                    lookup.erase(geoIndex);
                }
            };

    #pragma omp for schedule(static) nowait
            for (size_t i = 0; i < planeEvents->size(); ++i) {
                const auto &event = (*planeEvents)[i];
                if (event.plane.axisCoordinate != plane.axisCoordinate) {
                    insertIfAbsent(event.objIndex, event.plane.axisCoordinate < plane.axisCoordinate ? 0 : 1);
                } else if (event.type == PlaneEventType::planar) {
                    insertIfAbsent(event.objIndex, minSide ? 0 : 1);
                } else if (event.type == PlaneEventType::starting) {
                    insertIfAbsent(event.objIndex, 1);
                } else {
                    insertIfAbsent(event.objIndex, 0);
                }
            }
        }

        // Sequential merge — but the toggle semantics must be reapplied globally,
        // since the same geoIndex could land in different threads' local buffers.
        auto geometryMin = std::make_unique<ObjectIndexVector>();
        auto geometryMax = std::make_unique<ObjectIndexVector>();
        std::unordered_set<size_t> geometryMinLookup, geometryMaxLookup;
        geometryMin->reserve(planeEvents->size() / 4);
        geometryMax->reserve(planeEvents->size() / 4);

        auto mergeBucket = [](std::vector<ObjectIndexVector> &locals,
                              ObjectIndexVector &out, std::unordered_set<size_t> &seen) {
            for (auto &local: locals) {
                for (size_t geoIndex: local) {
                    if (!seen.contains(geoIndex)) {
                        seen.insert(geoIndex);
                        out.push_back(geoIndex);
                    } else {
                        seen.erase(geoIndex);
                    }
                }
            }
        };
        mergeBucket(localMin, *geometryMin, geometryMinLookup);
        mergeBucket(localMax, *geometryMax, geometryMaxLookup);

        return {std::move(geometryMin), std::move(geometryMax)};
    }
#endif

#if defined(KD_USE_TBB)
    #include <tbb/combinable.h>
    #include <tbb/parallel_for.h>

    ObjectIndexVectors<2> LogNSquaredPlane::generateGeometrySubsets(
            const std::shared_ptr<PlaneEventVector> &planeEvents,
            const Plane &plane, const bool minSide) {

        struct LocalBuckets {
            ObjectIndexVector min, max;
            std::unordered_set<size_t> minLookup, maxLookup;
        };

        // combinable<> lazily default-constructs one LocalBuckets per worker thread
        tbb::combinable<LocalBuckets> localData;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, planeEvents->size()),
                          [&](const tbb::blocked_range<size_t> &range) {
                              auto &local = localData.local();// thread-local reference, no lock

                              auto insertIfAbsent = [&](const size_t geoIndex, const uint8_t index) {
                                  auto &vector = index == 0 ? local.min : local.max;
                                  auto &lookup = index == 0 ? local.minLookup : local.maxLookup;
                                  if (!lookup.contains(geoIndex)) {
                                      lookup.insert(geoIndex);
                                      vector.push_back(geoIndex);
                                  } else {
                                      lookup.erase(geoIndex);
                                  }
                              };

                              for (size_t i = range.begin(); i != range.end(); ++i) {
                                  const auto &event = (*planeEvents)[i];
                                  if (event.plane.axisCoordinate != plane.axisCoordinate) {
                                      insertIfAbsent(event.objIndex, event.plane.axisCoordinate < plane.axisCoordinate ? 0 : 1);
                                  } else if (event.type == PlaneEventType::planar) {
                                      insertIfAbsent(event.objIndex, minSide ? 0 : 1);
                                  } else if (event.type == PlaneEventType::starting) {
                                      insertIfAbsent(event.objIndex, 1);
                                  } else {
                                      insertIfAbsent(event.objIndex, 0);
                                  }
                              }
                          });

        // Sequential merge
        auto geometryMin = std::make_unique<ObjectIndexVector>();
        auto geometryMax = std::make_unique<ObjectIndexVector>();
        std::unordered_set<size_t> geometryMinLookup, geometryMaxLookup;
        geometryMin->reserve(planeEvents->size() / 4);
        geometryMax->reserve(planeEvents->size() / 4);

        auto mergeBucket = [](const ObjectIndexVector &local, ObjectIndexVector &out,
                              std::unordered_set<size_t> &seen) {
            for (size_t geoIndex: local) {
                if (!seen.contains(geoIndex)) {
                    seen.insert(geoIndex);
                    out.push_back(geoIndex);
                } else {
                    seen.erase(geoIndex);
                }
            }
        };

        // combine_each visits every thread's local instance sequentially
        localData.combine_each([&](const LocalBuckets &local) {
            mergeBucket(local.min, *geometryMin, geometryMinLookup);
            mergeBucket(local.max, *geometryMax, geometryMaxLookup);
        });

        return {std::move(geometryMin), std::move(geometryMax)};
    }
#endif
}// namespace kdtree
