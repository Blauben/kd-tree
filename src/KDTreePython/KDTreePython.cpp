#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "KDTree/tree/KDTree.h"

namespace nb = nanobind;
using namespace nb::literals;

void printTree(const kdtree::KDTree &tree) {
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
            .def(nb::init<const std::vector<Vertex> &, const std::vector<IndexVector> &, const PlaneSelectionAlgorithm::Algorithm>(), "vertices"_a, "faces"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
            .def(nb::init<const std::tuple<std::vector<Vertex>, std::vector<IndexVector>> &, const PlaneSelectionAlgorithm::Algorithm>(), "polySource"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
            .def(nb::init<const std::string &, const std::string &, const PlaneSelectionAlgorithm::Algorithm>(), "nodeFilePath"_a, "faceFilePath"_a, "algorithm"_a = PlaneSelectionAlgorithm::Algorithm::LOG)
            .def("countIntersections", &KDTree::countIntersections, "origin"_a, "ray"_a)
            .def("getIntersections", [](KDTree &self, const Vertex &origin, const Vertex &ray) {
        std::set<Vertex> intersections{};
        self.getIntersections(origin, ray, intersections);
        return std::vector<Vertex>(intersections.begin(), intersections.end()); }, "origin"_a, "ray"_a)
            .def("prebuildTree", &KDTree::prebuildTree, nb::rv_policy::reference_internal)
            .def("printTree", [](const KDTree &tree) {
        std::ostringstream os;
        os << tree;
        nb::print(os.str().c_str()); })
            .def("__str__", [](const KDTree &tree) {
        std::ostringstream os;
        os << tree;
        return os.str(); });
}