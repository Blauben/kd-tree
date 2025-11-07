//
// Created by saruman on 07.11.25.
//
#pragma once

#include "KDTree/model/Plane.h"

namespace kdtree {
    /**
    * Used by {@link PlaneEvent} to position the face that generated the event relative to the generated plane.
    */
    enum class PlaneEventType {
        ending = 0,
        planar = 1,
        starting = 2,
    };

    /**
     * Generated when traversing the vector of faces and building their candidate planes.
     */
    struct PlaneEvent {
        PlaneEventType type;
        /**
         * The candidate plane suggested by the face included in this struct.
         */
        Plane plane;
        /**
         * The index of the face that generated this candidate plane.
         */
        unsigned int faceIndex;

        PlaneEvent(PlaneEventType type, Plane plane, unsigned faceIndex);

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
     * Extracts the indices of the triangle faces that are referenced by the given PlaneEvents.
     * @param events The PlaneEvents containing information about the faces.
     * @return A list of face indices.
     */
    ObjectIndexVector convertEventsToFaces(const std::variant<ObjectIndexVector, PlaneEventVector> &events);

    size_t countFaces(const std::variant<ObjectIndexVector, PlaneEventVector> &triangles);
}
