#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/vector.h>

#include "KDTree/tree/KDTree.h"

namespace nb = nanobind;
using namespace nb::literals;

void printTree(const kdtree::KDTree& tree) {
        std::ostringstream os;
        os << tree;
        nb::print(os.str().c_str());
}

NB_MODULE(KDTree_Python, m) {
    using namespace kdtree;
    nb::enum_<PlaneSelectionAlgorithm::Algorithm>(m, "PlaneSelectionAlgorithm")
    .value("LOG", PlaneSelectionAlgorithm::Algorithm::LOG)
    .value("LOGSQUARED", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)
    .value("QUADRATIC", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)
    .value("NOTREE", PlaneSelectionAlgorithm::Algorithm::NOTREE);
    nb::class_<KDTree>(m, "KDTree")
    .def(nb::init<const std::vector<Array3>&, const std::vector<IndexArray3>&, const PlaneSelectionAlgorithm::Algorithm>(), "vertices"_a, "faces"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
    .def(nb::init<const std::tuple<std::vector<Array3>, std::vector<IndexArray3>> &, const PlaneSelectionAlgorithm::Algorithm>(), "polySource"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
    .def(nb::init<const std::string&, const std::string&, const PlaneSelectionAlgorithm::Algorithm>(), "nodeFilePath"_a, "faceFilePath"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
    .def("countIntersections", &KDTree::countIntersections,"origin"_a, "ray"_a)
    .def("getFaceIntersections", [](KDTree& self, const Array3 &origin, const Array3 &ray) {
        std::set<Array3> intersections{};
        self.getFaceIntersections(origin, ray, intersections);
        return std::vector<Array3>(intersections.begin(), intersections.end());
    }, "origin"_a, "ray"_a)
    .def("prebuildTree", &KDTree::prebuildTree, nb::rv_policy::reference_internal)
    .def("printTree", [](const KDTree & tree) {
        std::ostringstream os;
        os << tree;
        nb::print(os.str().c_str());
    })
    .def("__str__", [](const KDTree & tree) {
        std::ostringstream os;
        os << tree;
        return os.str();
    });
}