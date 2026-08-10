#pragma once

#include <cstdint>

namespace kdtree::constants {

    // ============================================================================
    // Floating Point Arithmetic Constants
    // ============================================================================

    /**
     * The EPSILON used in the polyhedral gravity model to determine a radius around zero/ to use as slight offset.
     * @related Used to determine if a floating point number is equal to zero as threshold for rounding errors
     * @related Used for the sgn() function to determine the sign of a double value. Different compilers
     * produce different results if no EPSILON is applied for the comparison!
     */
    constexpr double EPSILON_ZERO_OFFSET = 1e-14;

    /**
     * This relative EPSILON is utilized ONLY for testing purposes to compare intermediate values to
     * Tsoulis' reference implementation Fortran.
     * It is used in the {@link kdtree::util::almostEqualRelative} function.
     *
     * @note While in theory no difference at all is observed when compiling this program on Linux using GCC on x86_64,
     *  the intermediate values change when the program is compiled in different environments.
     *  Hence, this solves the flakiness of the tests when on different plattforms
     */
    constexpr double EPSILON_ALMOST_EQUAL = 1e-10;

    /**
     * The maximal allowed ULP distance utilized for FloatingPoint comparisons using the
     * {@link kdtree::util::almostEqualUlps} function.
     *
     * @see https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
     */
    constexpr int MAX_ULP_DISTANCE = 4;

    // ============================================================================
    // Spatial/Tree Constants
    // ============================================================================

    /**
     * Number of dimensions for the polyhedron. Also corresponds to the number of elements of the {@link Direction} enum.
     */
    constexpr int DIMENSIONS = 3;

    /**
     * Maximum recursion depth for the KDTree. Used to limit the depth and size of the tree.
     */
    constexpr uint8_t MAX_RECURSION_DEPTH = 64;

    /**
     * Numerical tolerance for ray box intersections. Necessary to handle cases where the intersection ray is almost parallel to the split plane.
     */
    constexpr double EPSILON_NUMERICAL_TOLERANCE = 1e-9;

    // ============================================================================
    // Plane Selection Algorithm Constants
    // ============================================================================

    /**
     * Constant that describes the cost of traversing the KDTree by one step.
     */
    constexpr double TRAVERSE_STEP_COST = 1.0;

    /**
     * Constant that describes the cost of intersecting a ray and a single object.
     */
    constexpr double SHAPE_INTERSECTION_COST = 1.0;

    // ============================================================================
    // Ray-Geometry Intersection Constants
    // ============================================================================

    /**
     * Tolerance offset for ray-point intersection testing.
     * Used when intersecting rays with point geometry (spheres).
     */
    constexpr double EPSILON_RAY_POINT_OFFSET = 1e-2;

    /**
     * Tolerance offset for vertex containment checks in bounding boxes.
     * Used in needTreeRebuild() to check if vertices are within their expected bounding box.
     */
    constexpr double EPSILON_VERTEX_BOX_TOLERANCE = 0.5;

    /**
     * Threshold for the number of objects in a leaf node to determine whether to use Thrust on the device or host for intersection computations.
     * If the number of objects exceeds this threshold, Thrust will be used on the device (GPU) for parallel processing; otherwise, it will be used on the host (CPU).
     */
    constexpr size_t LEAF_THRUST_PARALLEL_THRESHOLD = 256;

    // ============================================================================
    // TetgenAdapter Constants
    // ============================================================================

    /**
     * The default exception message for TetgenAdapter errors
     */
    constexpr char TETGEN_DEFAULT_EXCEPTION_MSG[] =
            "The mesh was not read because of an error in Tetgen! This could indicate several "
            "issues, e. g. issues with the node assignment like they appear if either no nodes were "
            "read in at all or if no assignment was possible.";

    /**
     * Geometry index used as a unique identifier in debugging contexts to trace a specific geometry object through the KDTree construction and intersection processes.
     */
    constexpr unsigned long GEOMETRY_INDEX = 2124;

    /**
     * Maximum exponent difference for floating point operations in container utilities
     */
    constexpr int MAX_EXPONENT_DIFFERENCE = 50;

}// namespace kdtree::constants
