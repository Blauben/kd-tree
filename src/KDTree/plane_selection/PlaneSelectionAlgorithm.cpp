#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"

namespace kdtree {

    std::pair<const double, bool> PlaneSelectionAlgorithm::costForPlane(const Box &boundingBox, const Plane plane, const size_t shapesMin, const size_t shapesMax, const size_t shapesPlanar) {
        //Checks if the split plane is one of the shapes of the bounding box, if so the split is useless
        if (plane.axisCoordinate == boundingBox.minPoint[static_cast<int>(plane.orientation)] || plane.axisCoordinate == boundingBox.maxPoint[static_cast<int>(plane.orientation)]) {
            //will be discarded later because not splitting is cheaper (finitely many nodes!) than using this plane (infinite cost)
            return {std::numeric_limits<double>::infinity(), false};
        }
        //calculate parameters for Surface Area Heuristic (SAH): childBoxSurfaceAreas; number of contained shapes in each box
        auto [box1, box2] = boundingBox.splitBox(plane);
        //equalT are shapes lying in the plane (not in the boxes)
        const double surfaceAreaBounding = boundingBox.surfaceArea();
        const double surfaceArea1 = box1.surfaceArea();
        const double surfaceArea2 = box2.surfaceArea();
        //evaluate SAH: Include equalT once in each box and record option with minimum cost
        const double costLesser = constants::TRAVERSE_STEP_COST + constants::SHAPE_INTERSECTION_COST * (surfaceArea1 / surfaceAreaBounding * static_cast<double>(shapesMin + shapesPlanar) + surfaceArea2 / surfaceAreaBounding * static_cast<double>(shapesMax));
        const double costGreater = constants::TRAVERSE_STEP_COST + constants::SHAPE_INTERSECTION_COST * (surfaceArea1 / surfaceAreaBounding * static_cast<double>(shapesMin) + surfaceArea2 / surfaceAreaBounding * static_cast<double>(shapesMax + shapesPlanar));
        //if empty space is cut off, reduce cost by 20%
        const double factor = shapesMin == 0 || shapesMax == 0 ? 0.8 : 1;
        if (costLesser <= costGreater) {
            return {factor * costLesser, true};
        }
        //if empty space is cut off, reduce cost by 20%
        return {factor * costGreater, false};
    }
}// namespace kdtree