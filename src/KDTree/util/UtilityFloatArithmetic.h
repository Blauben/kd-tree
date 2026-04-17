/*
 * This file is taken from:
 *   https://https://github.com/esa/polyhedral-gravity-model
 *
 * Original authors: Schuhmacher, J., Blazquez, E., Gratl, F., Izzo, D., & Gómez, P.
 * License: GPL-3.0
 */

#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include "KDTree/util/Constants.h"

namespace kdtree::util {

    /**
     * Function for comparing closeness of two floating point numbers using ULP (Units in the Last Place) method.
     *
     * @tparam FloatType must be either double or float (ensured by static assertion)
     * @param lhs The left hand side floating point number to compare.
     * @param rhs The right hand side floating point number to compare.
     * @param ulpDistance The maximum acceptable ULP distance between the two floating points
     *      for which they would be considered near each other. This is optional and by default, it will be {@link MAX_ULP_DISTANCE}.
     *
     * @return true if the ULP distance between lhs and rhs is less than or equal to the provided ulpDistance value, otherwise, false.
     *  Returns true if both numbers are exactly the same. Returns false if the signs do not match.
     * @example The ULP distance between 3.0 and std::nextafter(3.0, INFINITY) would be 1,
     *      the ULP distance of 3.0 and std::nextafter(std::nextafter(3.0, INFINITY), INFINITY) would be 2, etc.
     * @see https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
     */
    template<typename FloatType>
    bool almostEqualUlps(FloatType lhs, FloatType rhs, int ulpDistance = constants::MAX_ULP_DISTANCE);

    /**
     * Function to check if two floating point numbers are relatively equal to each other within a given error range or tolerance.
     *
     * @tparam FloatType must be either double or float (ensured by static assertion)
     * @param lhs The first floating-point number to be compared.
     * @param rhs The second floating-point number to be compared.
     * @param epsilon The tolerance for comparison. Two numbers that are less than epsilon apart are considered equal.
     *                The default value is {@link EPSILON_ALMOST_EQUAL}.
     *
     * @return boolean value - Returns `true` if the absolute difference between `lhs` and `rhs` is less than or equal to
     *                         the relative error factored by the larger of the magnitude of `lhs` and `rhs`. Otherwise, `false`.
     * @see https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
     */
    template<typename FloatType>
    bool almostEqualRelative(FloatType lhs, FloatType rhs, double epsilon = constants::EPSILON_ALMOST_EQUAL);


}// namespace kdtree::util
