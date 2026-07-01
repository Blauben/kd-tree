/*
 * This file is taken from:
 *   https://https://github.com/esa/polyhedral-gravity-model
 *
 * Original authors: Schuhmacher, J., Blazquez, E., Gratl, F., Izzo, D., & Gómez, P.
 * License: GPL-3.0
 */

#include "KDTree/util/UtilityFloatArithmetic.h"


namespace kdtree::util {

    template<typename FloatType>
    bool almostEqualUlps(FloatType lhs, FloatType rhs, int ulpDistance) {
        static_assert(std::is_same_v<FloatType, float> || std::is_same_v<FloatType, double>,
                      "Template argument must be FloatType: Either float or double!");

        // In case the floats are equal in their representation, return true
        if (lhs == rhs) {
            return true;
        }

        // In case the signs mismatch, return false
        if ((lhs < static_cast<FloatType>(0.0) && rhs > static_cast<FloatType>(0.0)) ||
            (lhs > static_cast<FloatType>(0.0) && rhs < static_cast<FloatType>(0.0))) {
            return false;
        }


        if constexpr (std::is_same_v<FloatType, float>) {
            // In case of float, compute ULP distance by interpreting float as 32-bit integer
            return std::bit_cast<std::int32_t>(rhs) - std::bit_cast<std::int32_t>(lhs) <= ulpDistance;
        } else if constexpr (std::is_same_v<FloatType, double>) {
            // In case of double, compute ULP distance by interpreting double as 64-bit integer
            return std::bit_cast<std::int64_t>(rhs) - std::bit_cast<std::int64_t>(lhs) <= ulpDistance;
        }

        // Due to the static_assert above, this should not happen
        return false;
    }

    template<typename FloatType>
    bool almostEqualRelative(FloatType lhs, FloatType rhs, double epsilon) {
        const FloatType diff = std::abs(rhs - lhs);
        const FloatType largerValue = std::max(std::abs(rhs), std::abs(lhs));
        return diff <= largerValue * epsilon;
    }

}// namespace kdtree::util