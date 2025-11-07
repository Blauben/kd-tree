//
// Created by saruman on 07.11.25.
//
#include "KDTree/model/Plane.h"

namespace kdtree {
    Plane::Plane(const Array3 &point, Direction direction)
        : axisCoordinate(point[static_cast<int>(direction)]), orientation(direction) {
    }

    Plane::Plane(const double point, const Direction direction)
        : axisCoordinate(point), orientation(direction) {
    }

    std::ostream &operator<<(std::ostream &os, const Plane &plane) {
        os << "(" << plane.axisCoordinate << ", " << plane.orientation << ")";
        return os;
    }

    Array3 Plane::normal(const bool returnFlipped) const {
        using namespace util;
        return kdtree::normal(orientation) * (returnFlipped ? -1 : 1);
    }

    Array3 Plane::originPoint() const {
        Array3 point{0.0, 0.0, 0.0};
        point[static_cast<int>(orientation)] = axisCoordinate;
        return point;
    }

    double Plane::rayPlaneIntersection(const Array3 &origin, const Array3 &inverseRay) const {
        const auto &origin_coord = origin[static_cast<int>(orientation)];
        const auto &inverseRay_coord = inverseRay[static_cast<int>(orientation)];

        const auto t = (this->axisCoordinate - origin_coord) * inverseRay_coord;
        // NaN possible through 0./+-inf -> point lies on plane and ray is parallel to plane, t=0 should be returned
        // inverseRay_coord ray is +inf (happens during inverse calculation by 1./0.) if ray is parallel to plane -> If origin additionally not on the plane then algorithm returns +-inf . The sign gives information in which half-space defined by the plane the origin lies.
        return std::isnan(t) ? 0.0 : t;
    }


    bool Plane::operator==(const Plane &other) const {
        return std::fabs(axisCoordinate - other.axisCoordinate) < 1e-15 && orientation == other.orientation;
    }

    bool Plane::operator!=(const Plane &other) const {
        return !(*this == other);
    }
}