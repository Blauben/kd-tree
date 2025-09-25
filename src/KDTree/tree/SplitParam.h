#pragma once

#include <utility>

#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {
    //forward declaration
    class PlaneSelectionAlgorithm;

    /**
     * Helper struct to bundle important parameters required for splitting a Polyhedron for better readability.
     */
    struct SplitParam {
        /**
         * The vertices that compose the Polyhedron.
         */
        const PointVector &points;
        /**
         * Either an index list of faces that are included in the current bounding box of the KDTree or a list of PlaneEvents containing the information about thr bound faces. Important when building deeper levels of a KDTree.
         */
        PointIndexVector boundPoints;
        /**
         * The current bounding box that should be divided further by the KDTree.
         */
        Box boundingBox;
        /**
         * The direction in which the current bounding box should be divided by further.
         * Refer to {@link Plane} on how to interpret the Direction.
         */
        mutable Direction splitDirection;
        /**
         * The algorithm used to create new child TreeNodes after splitting the parent.
         */
        const std::shared_ptr<PlaneSelectionAlgorithm> planeSelectionStrategy;

        /**
         * Constructor that initializes all fields. Intended for the use with std::make_unique. See {@link SplitParam} fields for further information.
         *
         */
        SplitParam(const PointVector &points, const Box &boundingBox,
                   const Direction splitDirection,
                   const std::shared_ptr<PlaneSelectionAlgorithm> &planeSelectionStrategy)
            : points{points}, boundingBox{boundingBox},
              splitDirection{splitDirection}, planeSelectionStrategy{planeSelectionStrategy} {

        }

        /**
         * Constructor manually initializing boundFaces, used for testing.
         */
        SplitParam(const PointVector &points, PointIndexVector& boundPoints, const Box &boundingBox,
                   const Direction splitDirection,
                   const std::shared_ptr<PlaneSelectionAlgorithm> &planeSelectionStrategy)
            : points{points}, boundPoints{std::move(boundPoints)}, boundingBox{boundingBox},
              splitDirection{splitDirection}, planeSelectionStrategy{planeSelectionStrategy} {
        }
    };
} // namespace kdtree
