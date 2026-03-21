/**
 * @file KDTreePython.cpp
 * @brief Python bindings for the KDTree C++ library using nanobind.
 *
 * This file exposes the KDTree class and related enums to Python, allowing efficient spatial queries
 * and tree operations from Python code. The bindings include construction from vertices/faces, intersection queries,
 * and tree printing utilities.
 */

#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "KDTree/tree/KDTree.h"
#include "KDTree/model/Plane.h"
#include "KDTree/model/Box.h"

namespace nb = nanobind;
using namespace nb::literals;

/**
 * @brief Print the KDTree structure to the Python console.
 * @param tree Reference to the KDTree object.
 */
void printTree(const kdtree::KDTree &tree) {
    std::ostringstream os;
    os << tree;
    nb::print(os.str().c_str());
}

/**
 * @brief Python module definition for scikdtree.
 *
 * Exposes KDTree and PlaneSelectionAlgorithm to Python, including constructors and intersection methods.
 */
NB_MODULE(scikdtree, m) {
    using namespace kdtree;
    nb::class_<GeometryObject>(m, "GeometryObject")
        .def("vertices", &GeometryObject::getVertices, R"doc(Get the corner vertices of the geometry object.)doc")
        .def("vertex_indices", &GeometryObject::getIndexVector, R"doc(Get the indices of the corner vertices.)doc");
    // Expose Direction enum to Python
    nb::enum_<Direction>(m, "Direction")
        .value("X", Direction::X)
        .value("Y", Direction::Y)
        .value("Z", Direction::Z);
    // Expose Plane struct to Python
    nb::class_<Plane>(m, "Plane")
        .def(nb::init<>())
        .def_rw("axisCoordinate", &Plane::axisCoordinate)
        .def_rw("orientation", &Plane::orientation)
        .def("normal", &Plane::normal, "returnFlipped"_a = false, R"doc(Returns the normal vector for this plane.)doc")
        .def("originPoint", &Plane::originPoint, R"doc(Returns the origin point of the plane.)doc")
        .def("rayPlaneIntersection", &Plane::rayPlaneIntersection, "origin"_a, "inverseRay"_a, R"doc(Intersects a ray with the plane.)doc")
        .def("__eq__", &Plane::operator==);
    // Expose Box struct to Python
    nb::class_<Box>(m, "Box")
        .def(nb::init<>())
        .def_rw("minPoint", &Box::minPoint)
        .def_rw("maxPoint", &Box::maxPoint)
        .def("rayBoxIntersection", &Box::rayBoxIntersection, "origin"_a, "inverseRay"_a, R"doc(Calculates the intersection points of a ray and a box.)doc")
        .def("surfaceArea", &Box::surfaceArea, R"doc(Calculates the surface area of a box.)doc")
        .def("splitBox", &Box::splitBox, "plane"_a, R"doc(Splits this box into two new boxes.)doc")
        .def("clipToVoxel", &Box::clipToVoxel, "points"_a, R"doc(Clips vertices to this box.)doc");
    // Expose PlaneSelectionAlgorithm enum to Python
    nb::enum_<PlaneSelectionAlgorithm::Algorithm>(m, "PlaneSelectionAlgorithm")
        .value("LOG", PlaneSelectionAlgorithm::Algorithm::LOG)
        .value("LOGSQUARED", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)
        .value("QUADRATIC", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)
        .value("NOTREE", PlaneSelectionAlgorithm::Algorithm::NOTREE);
    // Expose KDTree class to Python
    nb::class_<KDTree>(m, "KDTree")
        .def(nb::init<const std::vector<Vertex> &, const std::vector<IndexVector> &, const PlaneSelectionAlgorithm::Algorithm>(), "vertices"_a, "faces"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
        .def(nb::init<const std::tuple<std::vector<Vertex>, std::vector<IndexVector>> &, const PlaneSelectionAlgorithm::Algorithm>(), "polySource"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
        .def(nb::init<const std::string &, const std::string &, const PlaneSelectionAlgorithm::Algorithm>(), "nodeFilePath"_a, "faceFilePath"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
        .def("countIntersections", &KDTree::countIntersections, "origin"_a, "ray"_a, R"doc(Count intersections of a ray with the KDTree.)doc")
        .def("getIntersections", [](KDTree &self, const Vertex &origin, const Vertex &ray) {
            std::set<Vertex> intersections{};
            self.getIntersections(origin, ray, intersections);
            return std::vector(intersections.begin(), intersections.end()); 
        }, "origin"_a, "ray"_a, R"doc(Get intersection points of a ray with the KDTree.)doc")
        .def("prebuildTree", &KDTree::prebuildTree, nb::rv_policy::reference_internal, R"doc(Prebuild the KDTree for faster queries.)doc")
        .def("printTree", [](const KDTree &tree) {
            std::ostringstream os;
            os << tree;
            nb::print(os.str().c_str());
        }, R"doc(Print the KDTree structure.)doc")
        .def("__str__", [](const KDTree &tree) {
            std::ostringstream os;
            os << tree;
            return os.str(); 
        }, R"doc(String representation of the KDTree.)doc")
        .def("planes",
             [m](KDTree &self) {
                auto [begin, end] = self.planeIterator();
                return nb::make_iterator(m, "plane_iterator", begin, end);
             }, R"doc(Iterate over the planes in the KDTree.)doc")
        .def("geometry", [m](KDTree &self) {
            auto [begin, end] = self.geometryIterator();
            return nb::make_iterator(m, "geometry_iterator", begin, end);
        }, R"doc(Get the geometry objects of the KDTree.)doc");
}