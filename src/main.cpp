#include "KDTree/Logging.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <CLI/CLI.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {
    using Algorithm = kdtree::PlaneSelectionAlgorithm::Algorithm;

    struct TreeStatistics {
        std::size_t geometryObjects = 0;
        std::size_t pointObjects = 0;
        std::size_t triangleObjects = 0;
        std::size_t otherObjects = 0;

        std::size_t nodeCount = 0;
        std::size_t splitNodeCount = 0;
        std::size_t leafNodeCount = 0;
        std::size_t maxDepth = 0;
        std::map<std::size_t, std::size_t> depthHistogram{};

        std::size_t planeCount = 0;
        std::array<std::size_t, 3> axisSplitCounts{};
        std::array<double, 3> minPlaneCoordinate{};
        std::array<double, 3> maxPlaneCoordinate{};
        std::array<double, 3> sumPlaneCoordinate{};

        double minBoxSurfaceArea = std::numeric_limits<double>::infinity();
        double maxBoxSurfaceArea = 0.0;
        double sumBoxSurfaceArea = 0.0;

        TreeStatistics() {
            minPlaneCoordinate.fill(std::numeric_limits<double>::infinity());
            maxPlaneCoordinate.fill(-std::numeric_limits<double>::infinity());
        }
    };

    [[nodiscard]] std::string toUpper(std::string value) {
        std::ranges::transform(value, value.begin(), [](const unsigned char c) {
            return static_cast<char>(std::toupper(c));
        });
        return value;
    }

    [[nodiscard]] Algorithm parseAlgorithm(std::string value) {
        value = toUpper(std::move(value));
        if (value == "LOG") {
            return Algorithm::LOG;
        }
        if (value == "LOGSQUARED") {
            return Algorithm::LOGSQUARED;
        }
        if (value == "QUADRATIC") {
            return Algorithm::QUADRATIC;
        }
        if (value == "NOTREE") {
            return Algorithm::NOTREE;
        }
        throw std::invalid_argument{"Unknown algorithm '" + value + "'. Use LOG, LOGSQUARED, QUADRATIC or NOTREE."};
    }

    void collectNodeStatistics(const std::shared_ptr<kdtree::TreeNode> &node, TreeStatistics &statistics) {
        if (node == nullptr) {
            return;
        }

        ++statistics.nodeCount;

        const std::size_t depth = kdtree::recursionDepth(node->nodeId);
        ++statistics.depthHistogram[depth];
        statistics.maxDepth = std::max(statistics.maxDepth, depth);

        if (const auto splitNode = std::dynamic_pointer_cast<kdtree::SplitNode>(node)) {
            ++statistics.splitNodeCount;
            collectNodeStatistics(splitNode->getChildNode(0), statistics);
            collectNodeStatistics(splitNode->getChildNode(1), statistics);
            return;
        }

        if (std::dynamic_pointer_cast<kdtree::LeafNode>(node) != nullptr) {
            ++statistics.leafNodeCount;
        }
    }

    void collectGeometryStatistics(kdtree::KDTree &tree, TreeStatistics &statistics) {
        auto [begin, end] = tree.geometryIterator();
        for (auto it = begin; it != end; ++it) {
            ++statistics.geometryObjects;
            const auto arity = it->getIndexVector().size();
            if (arity == 1) {
                ++statistics.pointObjects;
            } else if (arity == 3) {
                ++statistics.triangleObjects;
            } else {
                ++statistics.otherObjects;
            }
        }
    }
}

int main(int argc, char **argv) {
    using namespace kdtree;
    using namespace kdtree::util;

    CLI::App app{"Build a KDTree from Tetgen .node/.face files and run sample ray-intersection queries."};
    argv = app.ensure_utf8(argv);
    app.footer("Example:\n  KDTree --node resources/sphere_scaled-1732.node --face resources/sphere_scaled-1732.face --algorithm LOG");

    std::string nodeFile{};
    std::string faceFile{};
    std::string algorithmName{};
    app.add_option("-n,--node", nodeFile, "Path to .node file containing vertex data.")->check(CLI::ExistingFile)->required();
    app.add_option("-f,--face", faceFile, "Path to .face file containing triangle connectivity.")->check(CLI::ExistingFile)->required();
    app.add_option("-a,--algorithm", algorithmName, "Plane selection algorithm to use (LOG <default>, LOGSQUARED, QUADRATIC, NOTREE).")->default_val("LOG");

    CLI11_PARSE(app, argc, argv);

    try {
        LOG_INFO("Building KDTree from files: ", nodeFile, faceFile.empty() ? "" : " and ", faceFile);
        const Algorithm algorithm = parseAlgorithm(algorithmName);
        LOG_INFO("Using plane selection algorithm: ", algorithmName);

        KDTree tree{nodeFile, faceFile, algorithm};
        tree.prebuildTree();

        std::cout << tree << std::endl;

        TreeStatistics statistics{};
        if (const auto rootNode = tree.getRootNode(); rootNode != nullptr) {
            collectNodeStatistics(rootNode, statistics);
        }
        collectGeometryStatistics(tree, statistics);

        std::size_t planesNum = 0;
        auto [begin, end] = tree.planeIterator();
        for (auto it = begin; it != end; ++it) {
            const auto planeBox = *it;
            const auto &plane = planeBox.first;
            const auto &box = planeBox.second;

            std::cout << "Plane: " << planesNum++ << ", " << plane << ", Box: " << box << std::endl;

            const auto axisIndex = static_cast<std::size_t>(plane.orientation);
            ++statistics.planeCount;
            ++statistics.axisSplitCounts[axisIndex];
            statistics.minPlaneCoordinate[axisIndex] = std::min(statistics.minPlaneCoordinate[axisIndex], plane.axisCoordinate);
            statistics.maxPlaneCoordinate[axisIndex] = std::max(statistics.maxPlaneCoordinate[axisIndex], plane.axisCoordinate);
            statistics.sumPlaneCoordinate[axisIndex] += plane.axisCoordinate;

            const double surfaceArea = box.surfaceArea();
            statistics.minBoxSurfaceArea = std::min(statistics.minBoxSurfaceArea, surfaceArea);
            statistics.maxBoxSurfaceArea = std::max(statistics.maxBoxSurfaceArea, surfaceArea);
            statistics.sumBoxSurfaceArea += surfaceArea;
        }

        const auto averagePlaneCoordinate = [&statistics](const std::size_t axisIndex) {
            return statistics.axisSplitCounts[axisIndex] == 0
                       ? 0.0
                       : statistics.sumPlaneCoordinate[axisIndex] / static_cast<double>(statistics.axisSplitCounts[axisIndex]);
        };
        const auto averageBoxSurfaceArea = statistics.planeCount == 0
                                               ? 0.0
                                               : statistics.sumBoxSurfaceArea / static_cast<double>(statistics.planeCount);

        std::ostringstream summary;
        summary << std::fixed << std::setprecision(6);
        summary << "\nKDTree statistics\n";
        summary << "  Geometry objects: " << statistics.geometryObjects
                << " (triangles: " << statistics.triangleObjects
                << ", points: " << statistics.pointObjects
                << ", other: " << statistics.otherObjects << ")\n";
        summary << "  Tree nodes: " << statistics.nodeCount
                << " (split: " << statistics.splitNodeCount
                << ", leaf: " << statistics.leafNodeCount
                << ", max depth: " << statistics.maxDepth << ")\n";
        summary << "  Split planes: " << statistics.planeCount << "\n";
        summary << "  Axis distribution: X=" << statistics.axisSplitCounts[0]
                << ", Y=" << statistics.axisSplitCounts[1]
                << ", Z=" << statistics.axisSplitCounts[2] << "\n";
        summary << "  Plane coordinate ranges: "
                << "X[" << (statistics.axisSplitCounts[0] == 0 ? 0.0 : statistics.minPlaneCoordinate[0]) << " .. "
                << (statistics.axisSplitCounts[0] == 0 ? 0.0 : statistics.maxPlaneCoordinate[0]) << ", avg "
                << averagePlaneCoordinate(0) << "]"
                << ", Y[" << (statistics.axisSplitCounts[1] == 0 ? 0.0 : statistics.minPlaneCoordinate[1]) << " .. "
                << (statistics.axisSplitCounts[1] == 0 ? 0.0 : statistics.maxPlaneCoordinate[1]) << ", avg "
                << averagePlaneCoordinate(1) << "]"
                << ", Z[" << (statistics.axisSplitCounts[2] == 0 ? 0.0 : statistics.minPlaneCoordinate[2]) << " .. "
                << (statistics.axisSplitCounts[2] == 0 ? 0.0 : statistics.maxPlaneCoordinate[2]) << ", avg "
                << averagePlaneCoordinate(2) << "]\n";
        summary << "  Bounding box surface area: min="
                << (statistics.planeCount == 0 ? 0.0 : statistics.minBoxSurfaceArea)
                << ", max=" << (statistics.planeCount == 0 ? 0.0 : statistics.maxBoxSurfaceArea)
                << ", avg=" << averageBoxSurfaceArea << "\n";
        summary << "  Depth histogram:";
        if (statistics.depthHistogram.empty()) {
            summary << " none\n";
        } else {
            summary << '\n';
            for (const auto &[depth, count] : statistics.depthHistogram) {
                summary << "    depth " << depth << ": " << count << '\n';
            }
        }

        std::cout << summary.str();
        LOG_INFO("Demo finished successfully. Built KDTree with ", planesNum, " planes");
        return 0;
    } catch (const std::exception &error) {
        LOG_ERROR("KDTree failed: ", error.what());
        return 1;
    }
}