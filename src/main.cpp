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

    [[nodiscard]] std::string algorithmName(const Algorithm algorithm) {
        switch (algorithm) {
            case Algorithm::LOG:
                return "LOG";
            case Algorithm::LOGSQUARED:
                return "LOGSQUARED";
            case Algorithm::QUADRATIC:
                return "QUADRATIC";
            case Algorithm::NOTREE:
                return "NOTREE";
            default:
                return "UNKNOWN";
        }
    }

    void printUsage(const char *programName) {
        std::cout << "Usage: " << programName << " [mesh-prefix] [algorithm]\n"
                  << "  mesh-prefix: base path without suffix for .node/.face files\n"
                  << "               (default: resources/Eros_scaled-1732 with fallback search)\n"
                  << "  algorithm  : LOG | LOGSQUARED | QUADRATIC | NOTREE (default: LOG)\n\n"
                  << "Example:\n"
                  << "  " << programName << " resources/sphere_scaled-1732 LOG\n";
    }
}// namespace

int main(int argc, char **argv) {
    using namespace kdtree;
    using namespace kdtree::util;

    CLI::App app{"App description"};
    argv = app.ensure_utf8(argv);

    std::string node_file{};
    std::string face_file{};
    std::string algorithm{};
    app.add_option("-n,--node", node_file, "Path to .node file containing vertex data.")->;
    app.add_option("-f,--face", face_file, "Path to face file.");
    app.add_option("-a,--algorithm", algorithm, "Plane selection algorithm to use (LOG <default>, LOGSQUARED, QUADRATIC, NOTREE).")->default_val("LOG");

    app.add_flag("-h,--help", "Show this help message and exit.");

    CLI11_PARSE(app, argc, argv);

    try {
        if (argc > 1 && (std::string{argv[1]} == "-h" || std::string{argv[1]} == "--help")) {
            printUsage(argv[0]);
            return 0;
        }

        if (!std::filesystem::exists(nodeFile) || !std::filesystem::exists(faceFile)) {
            LOG_ERROR("Mesh files not found: ", nodeFile, " and ", faceFile);
            printUsage(argv[0]);
            return 1;
        }

        LOG_INFO("Building KDTree from files prefix: ", nodeFile, " and ", faceFile);
        LOG_INFO("Using plane selection algorithm: ", algorithmName(algorithm));

        KDTree tree{nodeFile, faceFile, algorithm};
        tree.prebuildTree();

        constexpr Vertex origin{200.0, 200.0, 200.0};
        const std::vector<Vertex> rayDirections{
                Vertex{-400.0, -400.0, -400.0},
                Vertex{-250.0, -190.0, -220.0},
                Vertex{-180.0, -260.0, -230.0}};

        for (size_t i = 0; i < rayDirections.size(); ++i) {
            const auto &ray = rayDirections[i];
            std::set<Vertex> intersections;
            tree.getIntersections(origin, ray, intersections);
            const size_t countViaFastPath = tree.countIntersections(origin, ray);

            std::cout << "Ray " << i + 1 << ": direction=["
                      << ray[0] << ", " << ray[1] << ", " << ray[2] << "]\n"
                      << "  intersections(set) : " << intersections.size() << "\n"
                      << "  intersections(count): " << countViaFastPath << "\n";

            if (!intersections.empty()) {
                const auto &first = *intersections.begin();
                std::cout << "  first hit          : ["
                          << first[0] << ", " << first[1] << ", " << first[2] << "]\n";
            }
        }

        LOG_INFO("Demo finished successfully.");
        return 0;
    } catch (const std::exception &error) {
        LOG_ERROR("KDTree failed: ", error.what());
        return 1;
    }
}