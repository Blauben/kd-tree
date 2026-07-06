#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <algorithm>
#include <benchmark/benchmark.h>
#include <string>
#include <vector>

namespace kdtree {

    static Vertex getFaceCentroid(const std::vector<Vertex> &vertices, const IndexVector &face) {
        using namespace kdtree::util;
        Vertex centroid{0, 0, 0};
        std::ranges::for_each(face, [&](const size_t vertexIndex) {
            centroid = centroid + vertices[vertexIndex];
        });
        return centroid / 3.0;
    }

    static std::vector<Vertex> getPolyhedralFaceCentroids(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &faces) {
        std::vector<Vertex> centroids;
        centroids.reserve(faces.size());
        std::ranges::transform(faces, std::back_inserter(centroids), [&vertices](const IndexVector &face) {
            return getFaceCentroid(vertices, face);
        });
        return centroids;
    }

    struct Meshes {
        std::vector<std::vector<Vertex>> vertices{};
        std::vector<std::vector<IndexVector>> faces{};
        std::vector<std::vector<Vertex>> centroids{};

        explicit Meshes(const std::vector<std::string> &filePaths) {
            std::ranges::for_each(filePaths, [this](const std::string &filePath) {
                const auto [fileVertices, fileFaces] = TetgenAdapter{buildCompleteFilePaths(filePath)}.getPolyhedralSource();
                centroids.push_back(getPolyhedralFaceCentroids(fileVertices, fileFaces));
                vertices.push_back(fileVertices);
                faces.push_back(fileFaces);
            });
        }

        std::tuple<const std::vector<Vertex> &, const std::vector<IndexVector> &, const std::vector<Vertex> &> operator[](const size_t index) const {
            return {vertices[index], faces[index], centroids[index]};
        }

        static std::vector<std::string> buildCompleteFilePaths(const std::string &filePath) {
            return {filePath + ".node", filePath + ".face"};
        }

        [[nodiscard]] long long size() const {
            return static_cast<long long>(vertices.size());
        }
    };

    Meshes erosMeshes{{"resources/Eros_scaled-1000", "resources/Eros_scaled-1732",
                       "resources/Eros_scaled-3000", "resources/Eros_scaled-5196",
                       "resources/Eros_scaled-9000", "resources/Eros_scaled-15588",
                       "resources/Eros_scaled-27000", "resources/Eros_scaled-46765",
                       "resources/Eros_scaled-81000", "resources/Eros_scaled-140296"}};


    Meshes sphereMeshes{{"resources/sphere_scaled-1000", "resources/sphere_scaled-1732",
                         "resources/sphere_scaled-3000", "resources/sphere_scaled-5196",
                         "resources/sphere_scaled-9000", "resources/sphere_scaled-15588",
                         "resources/sphere_scaled-27000", "resources/sphere_scaled-46765",
                         "resources/sphere_scaled-81000", "resources/sphere_scaled-140296"}};

    void BM_Eros_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.clear();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Sphere_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = sphereMeshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.clear();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Eros_Intersection_Tree_Twice(benchmark::State &state) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.clear();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Sphere_Intersection_Tree_Twice(benchmark::State &state) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = sphereMeshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.clear();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Eros_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            tree.prebuildTree();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Sphere_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = sphereMeshes[state.range(0)];
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            tree.prebuildTree();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    // eros mesh benchmarks
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronQuadratic",
                      PlaneSelectionAlgorithm::Algorithm::QUADRATIC)
            ->DenseRange(0, erosMeshes.size() - 3, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronLogSquared",
                      PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)
            ->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK(BM_Eros_Intersection_Tree_Twice)->Name("ErosPolyhedronSecondRun")->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree_Build, "ErosPolyhedronBuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(0, erosMeshes.size() - 3, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree_Build, "ErosPolyhedronBuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree_Build, "ErosPolyhedronBuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(0, erosMeshes.size() - 1, 1);

    // sphere mesh benchmarks
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronQuadratic",
                      PlaneSelectionAlgorithm::Algorithm::QUADRATIC)
            ->DenseRange(0, sphereMeshes.size() - 3, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronLogSquared",
                      PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)
            ->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK(BM_Sphere_Intersection_Tree_Twice)->Name("SpherePolyhedronSecondRun")->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree_Build, "SpherePolyhedronBuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree_Build, "SpherePolyhedronBuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree_Build, "SpherePolyhedronBuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(0, sphereMeshes.size() - 1, 1);
}// namespace kdtree

BENCHMARK_MAIN();
