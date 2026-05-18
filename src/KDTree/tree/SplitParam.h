#pragma once

#include "KDTree/tree/NodeRegister.h"
#include "KDTree/model/Box.h"
#include "KDTree/model/GeometryObject.h"
#include "KDTree/model/PlaneEvent.h"

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
        const std::vector<GeometryObject> &geometryObjects;

        /**
         * Either an index list of shapes that are included in the current bounding box of the KDTree or a list of PlaneEvents containing the information about thr bound shapes. Important when building deeper levels of a KDTree.
         */
        std::variant<ObjectIndexVector, PlaneEventVector> boundObjects;
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
         * The NodeRegister object located in the KDTree that is used to register and track node behavior throughout the tree.
         */
        NodeRegister& nodeRegister;

        /**
         * Constructor that initializes all fields. Intended for the use with std::make_unique. See {@link SplitParam} fields for further information.
         *
         */
        SplitParam(const std::vector<GeometryObject> &geometryObjects, const Box &boundingBox,
                   const Direction splitDirection,
                   const std::shared_ptr<PlaneSelectionAlgorithm> &planeSelectionStrategy, NodeRegister &nodeRegister)
            : geometryObjects{geometryObjects}, boundObjects{ObjectIndexVector(geometryObjects.size())}, boundingBox{boundingBox},
              splitDirection{splitDirection}, planeSelectionStrategy{planeSelectionStrategy}, nodeRegister{nodeRegister} {
            auto &indexList = std::get<ObjectIndexVector>(boundObjects);
            std::iota(indexList.begin(), indexList.end(), 0);
        }

        /**
         * Constructor manually initializing boundObjects, used for testing.
         */
        SplitParam(const std::vector<GeometryObject> &geometryObjects,
                   const std::variant<ObjectIndexVector, PlaneEventVector> &boundObjects, const Box &boundingBox,
                   const Direction splitDirection,
                   const std::shared_ptr<PlaneSelectionAlgorithm> &planeSelectionStrategy, NodeRegister &nodeRegister)
            : geometryObjects{geometryObjects}, boundObjects{boundObjects}, boundingBox{boundingBox},
              splitDirection{splitDirection}, planeSelectionStrategy{planeSelectionStrategy}, nodeRegister {nodeRegister} {
        }
    };
}// namespace kdtree
