#pragma once

#include <cassert>
#include <cstddef>
#include <iterator>
#include <limits>
#include <shared_mutex>
#include <tuple>
#include <utility>
#include <variant>

#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/util/Constants.h"

namespace kdtree {

    //forward declaration
    struct SplitParam;

    class PlaneSelectionAlgorithm {
    public:
        virtual ~PlaneSelectionAlgorithm() = default;
        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param splitParam specifies the space section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of shape sets with respective positions to the found plane. Refer to {@link ObjectIndexVectors<2>} for more information.
        */
        virtual std::tuple<Plane, double, std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>> findPlane(const SplitParam &splitParam) = 0;

        /**
         * Enum of available plane selection algorithms named after their runtimes.
         */
        enum class Algorithm {
            NOTREE,
            QUADRATIC,
            LOGSQUARED,
            LOG
        };

        /**
         * Takes subsets of shapes divided by a split plane and adds the shapes lying in the plane (planar) to one of the subsets.
         * @tparam ContainerVectors An array type that contains a fixed number of vectors specified through generics.
         * @param subsets The subsets of shapes divided by the split plane (min, max, planar).
         * @param minSideChosen Whether to add the planar shapes to the min side subset.
         * @return
         */
        template<template<size_t> typename ContainerVectors>
        ContainerVectors<2> addEqualPointsToSubset(ContainerVectors<3> subsets, const bool minSideChosen) {
            const size_t index = minSideChosen ? 0 : 1;
            subsets[index]->insert(subsets[index]->end(), subsets[2]->begin(), subsets[2]->end());
            return {(std::move(subsets[0])), (std::move(subsets[1]))};
        }

        /**
         * Class to evaluate candidate split planes and keep track of the optimal plane found so far.
         * @tparam CallbackArgs The argument types to be passed to the callback function when retrieving the geometry split through getPointsSplit().
         */
        template<typename... CallbackArgs>
        class OptimalPlane final {
            /** Mutex to synchronize access to the optimal plane data. Unused (and never locked) when
             * _useLocking is false.
            */
            std::shared_mutex _planeMutex{};

            /** Whether evaluatePlane() guards its accesses with _planeMutex. Set to false for
             * short-lived, thread-local instances (one per thread, never touched by any other thread)
             * that get combined into a single shared instance afterwards via planeWithLesserCost() - avoids
             * mutex overhead on every candidate evaluation, which otherwise dominates runtime under
             * heavy parallel use (e.g. SquaredPlane::findPlane's O(n^2) candidate search).
            */
            const bool _useLocking;

            /** The cost of the optimal plane found so far.
            */
            double _cost = std::numeric_limits<double>::infinity();

            /** The optimal plane found so far.
            */
            Plane _optPlane;

            /** The arguments to be passed to the callback function when retrieving the geometry split.
            */
            std::tuple<std::decay_t<CallbackArgs>...> _callbackArgs;

            /** The callback function to generate the geometry split for the optimal plane.
            */
            std::function<std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>(const OptimalPlane &, std::decay_t<CallbackArgs> &&...)> _boundGeometrySplit;

        public:
            /** The parameters of the scene to find the optimal plane for.
            */
            const SplitParam &splitParam;

            ~OptimalPlane() = default;

            /**
             * Constructs an OptimalPlane instance.
             * @param splitParam The parameters of the scene to find the optimal plane for.
             * @param boundPointsSplit The callback function to generate the geometry split for the optimal plane.
             * @param useLocking Whether to use locking when updating the optimal plane.
             */
            explicit OptimalPlane(const SplitParam &splitParam, const std::function<std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>>(const OptimalPlane &, CallbackArgs...)> &boundPointsSplit, const bool useLocking = true)
                : _useLocking(useLocking), _optPlane(0, splitParam.splitDirection), _boundGeometrySplit(boundPointsSplit), splitParam(splitParam) {
            }

            /**
             * Whether a candidate should be rejected in favor of the currently stored best: either it's
             * strictly worse, or it ties on cost but loses the axis-coordinate tie-break (used to
             * consistently build the same KDTree by eliminating indeterministic behavior should the cost
             * be equal - not important for functionality, but for testing purposes).
             * @param candidatePlane The candidate split plane to evaluate.
             * @param candidateCost The cost of the candidate split plane.
             */
            [[nodiscard]] bool isWorseThanCurrent(const Plane &candidatePlane, const double candidateCost) const {
                return (candidateCost == _cost && _optPlane.axisCoordinate < candidatePlane.axisCoordinate) || candidateCost > _cost;
            }

            /**
             * Overwrites the stored best plane/cost/callback-args with a new candidate. The caller is
             * responsible for holding whatever lock (if any) evaluatePlane()'s current mode requires.
             * @param candidatePlane The candidate split plane to store.
             * @param candidateCost The cost of the candidate split plane.
             * @param args The arguments to be passed to the callback function when retrieving the geometry split.
             */
            void setBest(const Plane &candidatePlane, const double candidateCost, CallbackArgs... args) {
                _cost = candidateCost;
                _optPlane = candidatePlane;
                //store the callback arguments for later use
                _callbackArgs = std::tuple<std::decay_t<CallbackArgs>...>(std::forward<CallbackArgs>(args)...);
            }

            /**
             * Evaluates a candidate split plane and updates the optimal plane if the candidate is better.
             * @param candidatePlane The candidate split plane to evaluate.
             * @param candidateCost The cost of the candidate split plane.
             * @param args The arguments to be passed to the callback function when retrieving the geometry split (should the candidate be optimal).
             */
            void evaluatePlane(const Plane &candidatePlane, const double candidateCost, CallbackArgs... args) {
                if (!_useLocking) {
                    if (isWorseThanCurrent(candidatePlane, candidateCost)) {
                        return;
                    }
                    setBest(candidatePlane, candidateCost, std::forward<CallbackArgs>(args)...);
                    return;
                }
                {
                    std::shared_lock readLock(_planeMutex);
                    if (isWorseThanCurrent(candidatePlane, candidateCost)) {
                        return;
                    }
                }
                std::unique_lock writeLock(_planeMutex);
                //re-checked since another thread may have updated _cost between the read lock above and this write lock.
                if (isWorseThanCurrent(candidatePlane, candidateCost)) {
                    setBest(candidatePlane, candidateCost, std::forward<CallbackArgs>(args)...);
                }
            }

            /**
                * Compares this OptimalPlane with another and returns the one with the lesser cost. If the costs are equal, the plane with the lesser axis coordinate is returned. Used to compare thread-local OptimalPlane instances and merge them into a single shared instance.
                * @param other The other OptimalPlane to compare with.
                * @return The OptimalPlane with the lesser cost.
             */
            friend OptimalPlane &planeWithLesserCost(OptimalPlane &one, OptimalPlane &two) {
                if (one._cost < two._cost || (one._cost == two._cost && one._optPlane.axisCoordinate < two._optPlane.axisCoordinate)) {
                    return one;
                }
                return two;
            }

            /** Retrieves the optimal split plane found so far.
            * @return The optimal split plane.
            */
            [[nodiscard]] Plane getOptimalPlane() const {
                return _optPlane;
            }

            /** Retrieves the cost of the optimal split plane found so far.
            * @return The cost of the optimal split plane.
            */
            [[nodiscard]] double getCost() const {
                return _cost;
            }

            /** Retrieves the geometry split for the optimal split plane found so far.
            * @return The geometry split for the optimal split plane.
            */
            std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>> getPointsSplit() {
                if (_cost == std::numeric_limits<double>::infinity()) {
                    return {};
                }
                return std::apply(
                        [this]<typename... Args>(Args &&...args) {
                            return _boundGeometrySplit(*this, std::forward<Args>(args)...);
                        },
                        std::move(_callbackArgs));
            }
        };

    protected:
        /**
       * Evaluates the cost function should the specified bounding box and it's shapes be divided by the specified plane. Used to evaluate possible split planes.
       * @param boundingBox the bounding box encompassing the scene to be split.
       * @param plane the candidate split plane to be evaluated.
       * @param shapesMin the number of shapes overlapping with the min side of the bounding box.
       * @param shapesMax the number of shapes overlapping with the max side of the bounding box.
       * @param shapesPlanar the number of shapes lying in the plane.
       * @param particleMode Whether the shapes the kd-tree is clustering are particles or geometric shapes (i.e. triangles).
       * @return A pair of: 1. the cost for performing intersection operations on the finalized tree later, should the KDTree be built using the specified split plane and the shape sets resulting through division by the plane.
       * 2. true if the planar shapes should be added to the min side of the bounding box.
       */
        static std::pair<const double, bool> costForPlane(const Box &boundingBox, Plane plane, size_t shapesMin, size_t shapesMax, size_t shapesPlanar, bool particleMode = false);
    };

}// namespace kdtree
