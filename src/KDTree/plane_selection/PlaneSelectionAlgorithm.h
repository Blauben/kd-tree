#pragma once

#include <cstddef>
#include <iterator>
#include <limits>
#include <tuple>
#include <utility>
#include <variant>

#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"

namespace kdtree {

    //forward declaration
    struct SplitParam;

    class PlaneSelectionAlgorithm {
    public:
        virtual ~PlaneSelectionAlgorithm() = default;
        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param splitParam specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexVectors<2>} for more information.
        */
        virtual std::tuple<Plane, double, std::variant<TriangleIndexVectors<2>, PlaneEventVectors<2>>> findPlane(const SplitParam &splitParam) = 0;

        enum class Algorithm {
            NOTREE,
            QUADRATIC,
            LOGSQUARED,
            LOG
        };

        template<template<size_t> typename ContainerVectors>
        ContainerVectors<2> addEqualPointsToSubset(ContainerVectors<3> subsets, bool minSideChosen) {
            const size_t index = minSideChosen ? 0 : 1;
            subsets[index]->insert(subsets[index]->end(), subsets[2]->begin(), subsets[2]->end());
            return {(std::move(subsets[0])), (std::move(subsets[1]))};
        }

        template<typename... CallbackArgs>
        class OptimalPlane final {
            std::mutex planeMutex{};

            double cost = std::numeric_limits<double>::infinity();

            Plane optPlane;

            std::tuple<std::decay_t<CallbackArgs>...> callbackArgs;

            std::function<std::variant<TriangleIndexVectors<2>, PlaneEventVectors<2>>(const OptimalPlane&, std::decay_t<CallbackArgs> &&...)> boundGeometrySplit;

        public:
            const SplitParam &splitParam;

            virtual ~OptimalPlane() = default;

            explicit OptimalPlane(const SplitParam &splitParam, const std::function<std::variant<TriangleIndexVectors<2>, PlaneEventVectors<2>>(const OptimalPlane&, CallbackArgs...)> &boundPointsSplit) : optPlane(0, splitParam.splitDirection), boundGeometrySplit(boundPointsSplit), splitParam(splitParam) {
            }

            void evaluatePlane(const Plane &candidatePlane, const double candidateCost, CallbackArgs... args) {
                std::unique_lock<std::mutex> lock(planeMutex);
                // this if-clause exists to consistently build the same KDTree (choose plane with lower coordinate) by eliminating indeterministic behavior should the cost be equal.
                // this is not important for functionality but for testing purposes
                if (candidateCost == cost && optPlane.axisCoordinate < candidatePlane.axisCoordinate) {
                    return;
                }
                if (candidateCost <= cost) {
                    cost = candidateCost;
                    optPlane = candidatePlane;
                    callbackArgs = std::tuple<std::decay_t<CallbackArgs>...>(std::forward<CallbackArgs>(args)...);
                }
            }

            [[nodiscard]] Plane getOptimalPlane() const {
                return optPlane;
            }

            [[nodiscard]] double getCost() const {
                return cost;
            }

            std::variant<TriangleIndexVectors<2>, PlaneEventVectors<2>> getPointsSplit() {
                return std::apply(boundGeometrySplit, std::tuple_cat(std::make_tuple(this), std::move(callbackArgs)));
            }
        };


        /**
        * Constant that describes the cost of traversing the KDTree by one step.
        */
        constexpr static double traverseStepCost{1.0};

        /**
        * Constant that describes the cost of intersecting a ray and a single object.
        */
        constexpr static double triangleIntersectionCost{1.0};

    protected:
        /**
       * Evaluates the cost function should the specified bounding box and it's faces be divided by the specified plane. Used to evaluate possible split planes.
       * @param boundingBox the bounding box encompassing the scene to be split.
       * @param plane the candidate split plane to be evaluated.
       * @param trianglesMin the number of triangles overlapping with the min side of the bounding box.
       * @param trianglesMax the number of triangles overlapping with the max side of the bounding box.
       * @param trianglesPlanar the number of triangles lying in the plane.
       * @return A pair of: 1. the cost for performing intersection operations on the finalized tree later, should the KDTree be built using the specified split plane and the triangle sets resulting through division by the plane.
       * 2. true if the planar triangles should be added to the min side of the bounding box.
       */
        static std::pair<const double, bool> costForPlane(Box boundingBox, Plane plane, size_t trianglesMin, size_t trianglesMax, size_t trianglesPlanar);
    };

}// namespace kdtree