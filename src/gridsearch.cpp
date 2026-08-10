// This executable brute-forces a small grid of tuning constants and times a
// representative build/query workload for each candidate.

#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"
#include "KDTree/tree/KdDefinitions.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace kdtree {

    struct Constants {
        int maxRecursionDepth;
        double shapeIntersectionCost;
        double traverseStepCost;
        double measurementTime{0};
    };

    std::string toString(const Constants &constants) {
        return "MAX_RECURSION_DEPTH: " + std::to_string(constants.maxRecursionDepth) +
               ", SHAPE_INTERSECTION_COST: " + std::to_string(constants.shapeIntersectionCost) +
               ", TRAVERSE_STEP_COST: " + std::to_string(constants.traverseStepCost) +
               ", MEASUREMENT_TIME: " + std::to_string(constants.measurementTime);
    }

    std::vector<Vertex> centroidsFromVerticesAndFaces(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &faces) {
        using namespace util;
        std::vector<Vertex> centroids;
        centroids.reserve(faces.size());
        for (const auto &face: faces) {
            Vertex centroid{0, 0, 0};
            for (const auto vertexIndex: face) {
                centroid = centroid + vertices[vertexIndex];
            }
            centroids.push_back(centroid / static_cast<double>(face.size()));
        }
        return centroids;
    }

    Constants evaluateCandidate(const std::vector<Vertex> &vertices,
                                const std::vector<IndexVector> &faces,
                                const std::vector<Vertex> &centroids,
                                const Constants &candidate) {
        using namespace util;
        constants::MAX_RECURSION_DEPTH = static_cast<uint8_t>(candidate.maxRecursionDepth);
        constants::SHAPE_INTERSECTION_COST = candidate.shapeIntersectionCost;
        constants::TRAVERSE_STEP_COST = candidate.traverseStepCost;

        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        const auto start = std::chrono::high_resolution_clock::now();
        KDTree tree{vertices, faces, PlaneSelectionAlgorithm::Algorithm::LOG};
        for (const auto &centroid: centroids) {
            intersections.clear();
            tree.getIntersections(origin, (centroid - origin) / 10.0, intersections);
        }
        const auto end = std::chrono::high_resolution_clock::now();

        Constants measured = candidate;
        measured.measurementTime = std::chrono::duration<double>(end - start).count();
        return measured;
    }

    std::vector<Constants> runGridSearch(const std::vector<Vertex> &vertices,
                                         const std::vector<IndexVector> &faces,
                                         const std::vector<Vertex> &centroids) {
        const std::tuple<double, double, int> recursionDepthValues{8, 256, 32};
        const std::tuple<double, double, int> shapeIntersectionCostValues{0.1, 10, 32};
        const std::tuple<double, double, int> traverseStepCostValues{0.1, 10, 32};

        std::vector<Constants> results;
        results.reserve((std::get<2>(recursionDepthValues) + 1) *
                        (std::get<2>(shapeIntersectionCostValues) + 1) *
                        (std::get<2>(traverseStepCostValues) + 1));

        for (int depth = 0; depth <= std::get<2>(recursionDepthValues); ++depth) {
            for (int shapeCost = 0; shapeCost <= std::get<2>(shapeIntersectionCostValues); ++shapeCost) {
                for (int traverseCost = 0; traverseCost <= std::get<2>(traverseStepCostValues); ++traverseCost) {
                    const Constants candidate{
                            static_cast<int>(std::get<0>(recursionDepthValues) + depth * (std::get<1>(recursionDepthValues) - std::get<0>(recursionDepthValues)) / std::get<2>(recursionDepthValues)),
                            std::get<0>(shapeIntersectionCostValues) + shapeCost * (std::get<1>(shapeIntersectionCostValues) - std::get<0>(shapeIntersectionCostValues)) / std::get<2>(shapeIntersectionCostValues),
                            std::get<0>(traverseStepCostValues) + traverseCost * (std::get<1>(traverseStepCostValues) - std::get<0>(traverseStepCostValues)) / std::get<2>(traverseStepCostValues)};
                    results.push_back(evaluateCandidate(vertices, faces, centroids, candidate));
                }
            }
        }

        std::ranges::sort(results, {}, &Constants::measurementTime);
        return results;
    }

}// namespace kdtree

int main() {
    using namespace kdtree;

    const auto [vertices, faces] = TetgenAdapter{{"resources/Eros_scaled-1000.node", "resources/Eros_scaled-1000.face"}}.getPolyhedralSource();
    const auto centroids = centroidsFromVerticesAndFaces(vertices, faces);
    const auto results = runGridSearch(vertices, faces, centroids);

    if (results.empty()) {
        std::cerr << "Grid search did not evaluate any candidates.\n";
        return 1;
    }

    const auto &best = results.front();
    std::cout << "Best candidate: " << toString(best) << '\n';
    std::cout << "Top 10 candidates:\n";
    for (size_t index = 0; index < std::min<size_t>(10, results.size()); ++index) {
        std::cout << index + 1 << ": " << toString(results[index]) << '\n';
    }

    return 0;
}
