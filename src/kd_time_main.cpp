#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <algorithm>
#include <benchmark/benchmark.h>
#include <string>
#include <vector>

namespace kdtree {

    static Array3 getFaceCentroid(const std::vector<Array3>& vertices, const IndexArray3 &face) {
        using namespace kdtree::util;
        Array3 centroid{0,0,0};
        std::for_each(face.cbegin(), face.cend(), [&](const size_t vertexIndex) {
            centroid = centroid + vertices[vertexIndex];
        });
        return centroid / 3.0;
    }

    static std::vector<Array3> getPolyhedralFaceCentroids(const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces) {
        std::vector<Array3> centroids;
        centroids.reserve(faces.size());
        std::transform(faces.begin(), faces.end(), std::back_inserter(centroids), [&vertices](const IndexArray3 &face) {
            return getFaceCentroid(vertices, face);
        });
        return centroids;
    }

    struct Meshes {
        std::vector<std::vector<Array3>> vertices{};
        std::vector<std::vector<IndexArray3>> faces{};
        std::vector<std::vector<Array3>> centroids{};

        explicit Meshes(const std::vector<std::string>& filePaths) {
            std::for_each(filePaths.cbegin(), filePaths.cend(), [this](const std::string& filePath) {
                const auto [fileVertices, fileFaces] = TetgenAdapter{buildCompleteFilePaths(filePath)}.getPolyhedralSource();
                centroids.push_back(getPolyhedralFaceCentroids(fileVertices, fileFaces));
                vertices.push_back(std::move(fileVertices));
                faces.push_back(std::move(fileFaces));
            });
        }

        std::tuple<std::vector<Array3>, std::vector<IndexArray3>, std::vector<Array3>> operator[](const size_t index) const {
            return std::make_tuple(vertices[index], faces[index], centroids[index]);
        }

        static std::vector<std::string> buildCompleteFilePaths(const std::string& filePath) {
            return {filePath + ".node", filePath + ".face"};
        }

        [[nodiscard]] long long size() const { return static_cast<long long>(vertices.size()); }
    };

    Meshes erosMeshes{{
            "polyhedral_files/Eros_scaled-1000", "polyhedral_files/Eros_scaled-1732",
            "polyhedral_files/Eros_scaled-3000", "polyhedral_files/Eros_scaled-5196",
            "polyhedral_files/Eros_scaled-9000", "polyhedral_files/Eros_scaled-15588",
            "polyhedral_files/Eros_scaled-27000", "polyhedral_files/Eros_scaled-46765",
            "polyhedral_files/Eros_scaled-81000", "polyhedral_files/Eros_scaled-140296"
        }};


    Meshes sphereMeshes{{
            "polyhedral_files/sphere_scaled-1000", "polyhedral_files/sphere_scaled-1732",
            "polyhedral_files/sphere_scaled-3000", "polyhedral_files/sphere_scaled-5196",
            "polyhedral_files/sphere_scaled-9000", "polyhedral_files/sphere_scaled-15588",
            "polyhedral_files/sphere_scaled-27000", "polyhedral_files/sphere_scaled-46765",
            "polyhedral_files/sphere_scaled-81000", "polyhedral_files/sphere_scaled-140296"
        }
    };

    void BM_Eros_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        constexpr Array3 origin{0,0,0};
        std::set<Array3> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            std::for_each(centroids.cbegin(), centroids.cend(), [&](const Array3& centroid) {
                tree.getFaceIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.erase(intersections.begin(), intersections.end());
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Sphere_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = sphereMeshes[state.range(0)];
        constexpr Array3 origin{0,0,0};
        std::set<Array3> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            std::for_each(centroids.cbegin(), centroids.cend(), [&](const Array3& centroid) {
                tree.getFaceIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.erase(intersections.begin(), intersections.end());
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Eros_Intersection_Tree_Twice(benchmark::State &state) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        constexpr Array3 origin{0,0,0};
        std::set<Array3> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            std::for_each(centroids.cbegin(), centroids.cend(), [&](const Array3& centroid) {
                tree.getFaceIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.erase(intersections.begin(), intersections.end());
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Sphere_Intersection_Tree_Twice(benchmark::State &state) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = sphereMeshes[state.range(0)];
        constexpr Array3 origin{0,0,0};
        std::set<Array3> intersections;
        for (auto _: state) {
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            std::for_each(centroids.cbegin(), centroids.cend(), [&](const Array3& centroid) {
                tree.getFaceIntersections(origin, (centroid - origin) / 10., intersections);
            });
            intersections.erase(intersections.begin(), intersections.end());
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Eros_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        for (auto _: state) {
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Sphere_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = sphereMeshes[state.range(0)];
        for (auto _: state) {
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    // eros mesh benchmarks
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->DenseRange(
        0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronQuadratic",
                      PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronLogSquared",
                      PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree, "ErosPolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(
        0, erosMeshes.size() - 1, 1);
    BENCHMARK(BM_Eros_Intersection_Tree_Twice)->Name("ErosPolyhedronSecondRun")->DenseRange(
        0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree_Build, "ErosPolyhedronBuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(
        0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree_Build, "ErosPolyhedronBuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(
        0, erosMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Eros_Intersection_Tree_Build, "ErosPolyhedronBuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(
        0, erosMeshes.size() - 1, 1);

    // sphere mesh benchmarks
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->DenseRange(
        0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronQuadratic",
                      PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronLogSquared",
                      PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree, "SpherePolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(
        0, sphereMeshes.size() - 1, 1);
    BENCHMARK(BM_Sphere_Intersection_Tree_Twice)->Name("SpherePolyhedronSecondRun")->DenseRange(
        0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree_Build, "SpherePolyhedronBuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(
        0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree_Build, "SpherePolyhedronBuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(
        0, sphereMeshes.size() - 1, 1);
    BENCHMARK_CAPTURE(BM_Sphere_Intersection_Tree_Build, "SpherePolyhedronBuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(
        0, sphereMeshes.size() - 1, 1);
} // namespace polyhedralGravity

BENCHMARK_MAIN();