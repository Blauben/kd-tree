#pragma once

#include "KDTree/model/Plane.h"

namespace kdtree {
    /**
    * Used by {@link PlaneEvent} to position the shape that generated the event relative to the generated plane.
    */
    enum class PlaneEventType {
        ending = 0,
        planar = 1,
        starting = 2,
    };

    /**
     * Generated when traversing the vector of shapes and building their candidate planes.
     */
    struct PlaneEvent {
        PlaneEventType type;
        /**
         * The candidate plane suggested by the shape included in this struct.
         */
        Plane plane;
        /**
         * The index of the shape that generated this candidate plane.
         */
        unsigned int objIndex;

        PlaneEvent(PlaneEventType type, Plane plane, unsigned objIndex);

        PlaneEvent() = default;

        /**
         * Less operator used for sorting an PlaneEvent vector.
         * @param other the PlaneEvent to compare this to.
         * @return true if this should precede the other argument.
         */
        bool operator<(const PlaneEvent &other) const;

        /**
         *Equality operator used for testing purposes
         */
        bool operator==(const PlaneEvent &other) const;
    };

    /**
     * A list of PlaneEvents.
    */
    using PlaneEventVector = std::vector<PlaneEvent>;

    /**
    * An array of PlaneEventLists.
    */
    template<size_t Number>
    using PlaneEventVectors = std::array<std::unique_ptr<PlaneEventVector>, Number>;

    /**
     * Extracts the indices of the shapes that are referenced by the given PlaneEvents.
     * @param events The PlaneEvents containing information about the shapes.
     * @return A list of shape indices.
     */
    ObjectIndexVector convertEventsToGeometry(const std::variant<ObjectIndexVector, PlaneEventVector> &events);

    /**
     * Calculate the number of distinct geometryObjects encoded in the 'geometry' argument.
     * @param geometry a collection of encoded GeometryObjects (indices or PlaneEvents)
     * @return the number of distinct geometryObjects
     */
    size_t countGeometryObjects(const std::variant<ObjectIndexVector, PlaneEventVector> &geometry);
}// namespace kdtree
