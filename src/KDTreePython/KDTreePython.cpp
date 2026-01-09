/**
 * @file KDTreePython.cpp
 * @brief Python bindings for the KDTree C++ library using nanobind.
 *
 * This file exposes the KDTree class and related enums to Python, allowing efficient spatial queries
 * and tree operations from Python code. The bindings include construction from vertices/faces, intersection queries,
 * and tree printing utilities.
 */

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "KDTree/tree/KDTree.h"

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
 * @brief Python module definition for KDTree_Python.
 *
 * Exposes KDTree and PlaneSelectionAlgorithm to Python, including constructors and intersection methods.
 */
NB_MODULE(KDTree_Python, m) {
    using namespace kdtree;
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
        }, R"doc(String representation of the KDTree.)doc");
}