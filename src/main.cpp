#include "KDTree/Logging.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <CLI/CLI.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace {
    using Algorithm = kdtree::PlaneSelectionAlgorithm::Algorithm;

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

}// namespace

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

        // TODO: plot tree
        std::cout << tree << std::endl;
        LOG_INFO("Demo finished successfully.");
        return 0;
    } catch (const std::exception &error) {
        LOG_ERROR("KDTree failed: ", error.what());
        return 1;
    }
}