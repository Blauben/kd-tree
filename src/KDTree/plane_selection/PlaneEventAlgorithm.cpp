#include "KDTree/plane_selection/PlaneEventAlgorithm.h"

namespace kdtree {
    TriangleCounter::TriangleCounter(const size_t dimensionCount, const std::array<size_t, 3> &initialValues)
        : dimensionTriangleValues(dimensionCount, initialValues) {
        if (dimensionCount == 0) {
            throw std::invalid_argument("Dimension count must be greater than zero");
        }
    }

    void TriangleCounter::updateMax(Direction direction, const size_t p_planar, const size_t p_end) {
        dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(1) -= p_planar +
                p_end;
    }

    void TriangleCounter::updateMin(Direction direction, const size_t p_planar, const size_t p_start) {
        dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(0) += p_planar +
                p_start;
    }

    void TriangleCounter::setPlanar(Direction direction, const size_t p_planar) {
        dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(2) = p_planar;
    }

    size_t TriangleCounter::getMin(Direction direction) const {
        return dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(0);
    }

    size_t TriangleCounter::getMax(Direction direction) const {
        return dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(1);
    }

    size_t TriangleCounter::getPlanar(Direction direction) const {
        return dimensionTriangleValues.at(static_cast<size_t>(direction) % dimensionTriangleValues.size()).at(2);
    }
} // namespace kdtree
