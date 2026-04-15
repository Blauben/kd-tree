#pragma once

#include "KDTree/model/Plane.h"
#include "KDTree/tree/KdDefinitions.h"

#include <stdexcept>

namespace kdtree {
    /**
     * Defines a rectangular box by taking two opposite corner points. First is the point closest to the origin and second is the point farthest away.
     */
    struct Box {
        /**
         * The point closer to the origin, thus minimal
         */
        Vertex minPoint;
        /**
         * The point further away from the origin, thus maximal.
         */
        Vertex maxPoint;

        /**
         * Calculates the intersection points of a ray and a box.
         * @param origin The origin of the ray.
         * @param inverseRay The inverse ray direction vector of the ray to be intersected (used for faster calculations).
         * @return Parameters t of the equation $ intersection_point = origin + t * ray $ for the entry and exit intersection points.
         */
        [[nodiscard]] std::pair<double, double> rayBoxIntersection(const Vertex &origin, const Vertex &inverseRay) const;

        /**
        * Calculates the surface area of a box.
        * @return the surface area
        */
        [[nodiscard]] double surfaceArea() const;

        /**
       * Splits this box into two new boxes.
       * @param plane the plane by which to split the original box.
       * @return a pair of boxes that result by splitting this box.
       */
        [[nodiscard]] std::pair<Box, Box> splitBox(const Plane &plane) const;

        /**
        * Finds the minimal bounding box for a set of vertices.
        * @param vertices the set of vertex coordinates for which to find the box
        * @return the bounding box {@link Box}
        */
        template<typename Container>
        static Box getBoundingBox(const Container &vertices) {
            using namespace util;
            return Box(findMinMaxCoordinates<Container>(vertices));
        }

        /**
        * Takes points of a shape of a polyhedron and clips them to this box. If all the points lie in the box no changes are made but if points lie outside of the box they are linearly interpolated onto the box.
        * Uses the Sutherland-Hodgman-Algorithm.
        * @param points The corner points of the shape to be clipped.
        * @return The new corner points of the clipped shape.
        */
        [[nodiscard]] std::vector<Vertex> clipToVoxel(const std::vector<Vertex> &points) const;

        explicit Box(const std::pair<Vertex, Vertex> &pair);
        Box();

        friend std::ostream &operator<<(std::ostream &os, const Box &box);

    private:
        /**
         * Takes a plane and a set of vertices and clips them accordingly.
         * Used as a sub procedure by the Sutherland-Hodgman-Algorithm.
         * @param plane The plane to split the vertices by
         * @param flipPlaneNormal Specifies which side of the plane is inside (In the direction or opposite of the plane normal).
         * @param source The vertices to be transformed to lie on the inside of the plane.
         * @param dest The transformed vertices.
         */
        static void clipToVoxelPlane(const Plane &plane, bool flipPlaneNormal, const std::vector<Vertex> &source, std::vector<Vertex> &dest);
    };
}// namespace kdtree
