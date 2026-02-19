#include "KDTree/model/Box.h"

namespace kdtree {
        Box::Box(const std::pair<Array3, Array3> &pair)
        : minPoint{pair.first}, maxPoint{pair.second} {
    }

    Box::Box()
        : minPoint{0.0, 0.0, 0.0}, maxPoint{0.0, 0.0, 0.0} {
    }

    std::pair<double, double> Box::rayBoxIntersection(const Array3 &origin, const Array3 &inverseRay) const {
        //calculate the parameter t in $ origin + t * ray = point $
        const auto lambdaIntersectSlabPoint = [&origin, &inverseRay](const Array3 &point) {
            using namespace util;
            std::array<double, 3> result{};
            for (const auto &direction: ALL_DIRECTIONS) {
                result[static_cast<size_t>(direction)] = Plane(point, direction).rayPlaneIntersection(origin, inverseRay);
            }
            return result;
        };
        //intersections with slabs defined through minPoint
        auto [tx_min, ty_min, tz_min] = lambdaIntersectSlabPoint(minPoint);
        //intersections with slabs defined through maxPoint
        auto [tx_max, ty_max, tz_max] = lambdaIntersectSlabPoint(maxPoint);

        const auto assignEnterExitValue = [](const auto t_min, const auto t_max,
                                             const auto rayDir) -> std::pair<double, double> {
            // if ray is shot in positive ray direction then minPoint slab is hit before max point slab -> min point slab holds the entry point and max point slab the exit point
            // otherwise max point slab is hit first and then min point slab after
            return rayDir < 0 ? std::make_pair(t_max, t_min) : std::make_pair(t_min, t_max);
        };
        //return the parameters in ordered by '<'
        auto [tx_enter, tx_exit] = assignEnterExitValue(tx_min, tx_max, inverseRay[0]);
        auto [ty_enter, ty_exit] = assignEnterExitValue(ty_min, ty_max, inverseRay[1]);
        auto [tz_enter, tz_exit] = assignEnterExitValue(tz_min, tz_max, inverseRay[2]);

        //calculate the point where all slabs have been entered: t_enter
        const double t_enter{std::max(tx_enter, std::max(ty_enter, tz_enter))};
        //calculates the point where the first slab has been exited: t_exit
        const double t_exit{std::min(tx_exit, std::min(ty_exit, tz_exit))};

        return {t_enter, t_exit};
    }

    double Box::surfaceArea() const {
        const double width = std::abs(maxPoint[0] - minPoint[0]);
        const double length = std::abs(maxPoint[1] - minPoint[1]);
        const double height = std::abs(maxPoint[2] - minPoint[2]);
        return 2 * (width * length + width * height + length * height);
    }

    std::pair<Box, Box> Box::splitBox(const Plane &plane) const {
        //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
        Box box1{*this};
        Box box2{*this};
        const Direction &axis{plane.orientation};
        //Shift edges of the boxes to match the plane
        box1.maxPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        box2.minPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        return std::make_pair(box1, box2);
    }

    std::vector<Array3> Box::clipToVoxel(const std::vector<Array3> &points) const {
        using namespace util;
        //use clipped as the input vector because the inner for loop swaps input and clipped each iteration,
        //since each iteration needs the output of the previous iteration as input.
        std::vector clipped(points.cbegin(), points.cend());
        std::vector<Array3> input{};
        input.reserve(points.size());
        //every plane defined by the maxPoint has to flip its normal because the normals have to point inside the bounding box.
        bool flipPlane = false;
        for (const Direction direction: ALL_DIRECTIONS) {
            const auto directionPlanes = {Plane(minPoint, direction), Plane(maxPoint, direction)};
            for (const auto &plane: directionPlanes) {
                std::swap(input, clipped);
                clipToVoxelPlane(plane, flipPlane, input, clipped);
                input.clear();
                flipPlane = !flipPlane;
            }
        }
        return clipped;
    }

    void Box::clipToVoxelPlane(const Plane &plane, const bool flipPlaneNormal, const std::vector<Array3> &source,
                               std::vector<Array3> &dest) {
        using namespace util;
        //the distance is interpreted in the normal direction, negative values are in opposite direction of the normal.
        auto distanceMeasures = [&plane, &flipPlaneNormal](
                                        const Array3 &point) -> double {
            // works for arbitrary plane orientations:
            // return dot(point - plane.originPoint(), plane.normal(flipPlaneNormal));
            // only works for axis aligned planes:
            return (point[static_cast<int>(plane.orientation)] - plane.axisCoordinate) * (flipPlaneNormal ? -1. : 1.);
        };
        static constexpr auto isInside = [](const double distance) { return distance >= 0.0; };
        static constexpr auto intersectionPoint = [](const Array3 &from, const Array3 &to, const double distanceFrom,
                                                     const double distanceTo) {
            // solve for t in $ [(t * from + (1-t) * to ) - origin] * normal = 0 $
            // equation explained: search for a point on the plane defined by $ (point - origin) * normal $, where point is linearly interpolated using vectors from and to.
            const double t{distanceTo / (distanceTo - distanceFrom)};
            return from * t + to * (1.0 - t);
        };
        for (size_t i{0}; i < source.size(); i++) {
            const Array3 &from{source[i]};
            const Array3 &to{source[(i + 1) % source.size()]};
            // $ (from - origin) * normal = cos alpha * |from - origin| * |normal| = cos alpha * |from - origin| * 1 $ ^= distance of from to the plane in the direction of the normal.
            const auto distanceFrom = distanceMeasures(from);
            const auto distanceTo = distanceMeasures(to);
            if (isInside(distanceFrom) && isInside(distanceTo)) {
                dest.push_back(to);
            } else if (isInside(distanceFrom) && !isInside(distanceTo)) {
                dest.emplace_back(intersectionPoint(from, to, distanceFrom, distanceTo));
            } else if (!isInside(distanceFrom) && isInside(distanceTo)) {
                dest.emplace_back(intersectionPoint(from, to, distanceFrom, distanceTo));
                dest.push_back(to);
            } else if (!isInside(distanceFrom) && !isInside(distanceTo) && source.size() == 1) {
                throw std::runtime_error("Algorithmic Error! Single Point outside the voxel passed to Box::clipToVoxel");
            }
        }
    }
}