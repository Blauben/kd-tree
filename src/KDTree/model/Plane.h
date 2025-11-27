#pragma once

#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {
    /**
         * Defines a plane that is parallel to one of the coordinate planes, by taking the fixed axis coordinate value for the plane and the coordinate index ({@link Direction}) that is fixed for every \
         * point on the plane.
         *
         * E.g. Specifying 0.0 and Direction::X would describe the YZ plane that goes through the origin. The direction is equivalent to the coordinate that is 1 in the normal vector of the plane, the others are 0.
         */
    struct Plane {
        /**
         * Each point lying on the plane has to have this value in the dimension specified in the orientation parameter.
         */
        double axisCoordinate;
        /**
         * Specifies which coordinate dimension is fixed for every point on the plane.
         */
        Direction orientation;

        /**
        * Returns the normal vector for this plane.
        * @param returnFlipped Whether to return the normal pointing in the opposite direction.
        * @return The normal vector.
        */
        [[nodiscard]] Vertex normal(bool returnFlipped = false) const;

        /**
         * Returns the origin point of a plane, meaning a point that lies on the plane.
         * @return The origin point.
         */
        [[nodiscard]] Vertex originPoint() const;

        /**
        * Intersects a ray with the splitPlane.
        * @param origin The point where the ray originates from.
        * @param inverseRay The inverse ray direction vector of the ray to be intersected (used for faster calculations).
        * @return Returns the t parameter for the intersection point, with t being from the equation $intersection_point = orig + t * ray$. t is +-infinity if no intersection point is present.
        */
        [[nodiscard]] double rayPlaneIntersection(const Vertex &origin, const Vertex &inverseRay) const;

        /**
        * Equality operator used for testing purposes
        */
        bool operator==(const Plane &other) const;

        /**
        * Inequality operator used for testing purposes
        */
        bool operator!=(const Plane &other) const;

        /**
         * Used to print to a Plane an ostream.
         * @param os The stream to print to.
         * @param plane The plane that is printed.
         * @return The ostream for chaining calls.
         */
        friend std::ostream &operator<<(std::ostream &os, const Plane &plane);

        Plane() = default;
        /**
         * Constructs a Plane from a point (vertex) and orientation
         * @param point The point that lies on the plane.
         * @param direction The orientation in which the plane spans.
         */
        Plane(const Vertex &point, Direction direction);

        /**
         * Constructs a plane from a coordinate on the axis that is given by the plane orientation.
         * @param point The plane anchor point coordinate.
         * @param direction The orientation in which the plane spans.
         */
        Plane(double point, Direction direction);
    };
}