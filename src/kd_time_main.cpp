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

    struct SingleFileMeshes {
        std::vector<std::vector<Vertex>> vertices{};
        std::vector<std::vector<IndexVector>> faces{};
        std::vector<std::vector<Vertex>> centroids{};

        explicit SingleFileMeshes(const std::vector<std::string> &filePaths) {
            std::ranges::for_each(filePaths, [this](const std::string &filePath) {
                const auto [fileVertices, fileFaces] = TetgenAdapter{{filePath}}.getPolyhedralSource();
                centroids.push_back(getPolyhedralFaceCentroids(fileVertices, fileFaces));
                vertices.push_back(fileVertices);
                faces.push_back(fileFaces);
            });
        }

        std::tuple<const std::vector<Vertex> &, const std::vector<IndexVector> &, const std::vector<Vertex> &> operator[](const size_t index) const {
            return {vertices[index], faces[index], centroids[index]};
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

    SingleFileMeshes a8567Mesh{{"resources/a8567.tab.ply"}};
    SingleFileMeshes comet67PMesh{{"resources/67P_ESA_NAVCAM_Jul2015data_256k.ply"}};
    SingleFileMeshes toutatisMesh{{"resources/4179toutatis.tab.ply"}};
    SingleFileMeshes itokawaObjectMesh{{"resources/Object 25143_Itokawa_200k.ply"}};
    SingleFileMeshes hartley2Mesh{{"resources/hartley2_2012_cart.ply"}};
    SingleFileMeshes shapeSfmMesh{{"resources/SHAPE_SFM_3M_v20180804.ply"}};
    SingleFileMeshes mu69Mesh{{"resources/MU69_Merged.ply"}};

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

    void BM_Single_File_Mesh_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm, const SingleFileMeshes &meshes) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = meshes[state.range(0)];
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

    void BM_Single_File_Mesh_Intersection_Tree_Twice(benchmark::State &state, const SingleFileMeshes &meshes) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = meshes[state.range(0)];
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

    void BM_Single_File_Mesh_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm, const SingleFileMeshes &meshes) {
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = meshes[state.range(0)];
        for (auto _: state) {
            KDTree tree{vertices, faces, algorithm};
            tree.prebuildTree();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    // eros mesh benchmarks
#define REGISTER_POLYHEDRON_MESH_BENCHMARKS(prefix, mesh, intersectionBenchmark, twiceBenchmark, buildBenchmark)                                                                              \
    BENCHMARK_CAPTURE(intersectionBenchmark, prefix "/NoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->Name(prefix "/NoTree")->DenseRange(0, mesh.size() - 1, 1);                        \
    BENCHMARK_CAPTURE(intersectionBenchmark, prefix "/Quadratic", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Name(prefix "/Quadratic")->DenseRange(0, mesh.size() - 3, 1);               \
    BENCHMARK_CAPTURE(intersectionBenchmark, prefix "/LogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Name(prefix "/LogSquared")->DenseRange(0, mesh.size() - 1, 1);            \
    BENCHMARK_CAPTURE(intersectionBenchmark, prefix "/Log", PlaneSelectionAlgorithm::Algorithm::LOG)->Name(prefix "/Log")->DenseRange(0, mesh.size() - 1, 1);                                 \
    BENCHMARK(twiceBenchmark)->Name(prefix "/SecondRun")->DenseRange(0, mesh.size() - 1, 1);                                                                                                  \
    BENCHMARK_CAPTURE(buildBenchmark, prefix "/BuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Name(prefix "/BuildTreeSquared")->DenseRange(0, mesh.size() - 3, 1);        \
    BENCHMARK_CAPTURE(buildBenchmark, prefix "/BuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Name(prefix "/BuildTreeLogSquared")->DenseRange(0, mesh.size() - 1, 1); \
    BENCHMARK_CAPTURE(buildBenchmark, prefix "/BuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->Name(prefix "/BuildTreeLog")->DenseRange(0, mesh.size() - 1, 1);

    REGISTER_POLYHEDRON_MESH_BENCHMARKS("ErosPolyhedron", erosMeshes, BM_Eros_Intersection_Tree, BM_Eros_Intersection_Tree_Twice, BM_Eros_Intersection_Tree_Build)

    // sphere mesh benchmarks
    REGISTER_POLYHEDRON_MESH_BENCHMARKS("SpherePolyhedron", sphereMeshes, BM_Sphere_Intersection_Tree, BM_Sphere_Intersection_Tree_Twice, BM_Sphere_Intersection_Tree_Build)

#undef REGISTER_POLYHEDRON_MESH_BENCHMARKS

    // single-file mesh benchmarks
#define REGISTER_SINGLE_FILE_MESH_BENCHMARK(prefix, mesh)                                                                                                                                                                        \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree, prefix "/NoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE, mesh)->Name(prefix "/NoTree")->DenseRange(0, mesh.size() - 1, 1);                                     \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree, prefix "/Quadratic", PlaneSelectionAlgorithm::Algorithm::QUADRATIC, mesh)->Name(prefix "/Quadratic")->DenseRange(0, mesh.size() - 1, 1);                            \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree, prefix "/LogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED, mesh)->Name(prefix "/LogSquared")->DenseRange(0, mesh.size() - 1, 1);                         \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree, prefix "/Log", PlaneSelectionAlgorithm::Algorithm::LOG, mesh)->Name(prefix "/Log")->DenseRange(0, mesh.size() - 1, 1);                                              \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree_Twice, prefix "/SecondRun", mesh)->Name(prefix "/SecondRun")->DenseRange(0, mesh.size() - 1, 1);                                                                     \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree_Build, prefix "/BuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC, mesh)->Name(prefix "/BuildTreeSquared")->DenseRange(0, mesh.size() - 3, 1);        \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree_Build, prefix "/BuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED, mesh)->Name(prefix "/BuildTreeLogSquared")->DenseRange(0, mesh.size() - 1, 1); \
    BENCHMARK_CAPTURE(BM_Single_File_Mesh_Intersection_Tree_Build, prefix "/BuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG, mesh)->Name(prefix "/BuildTreeLog")->DenseRange(0, mesh.size() - 1, 1);

    REGISTER_SINGLE_FILE_MESH_BENCHMARK("A8567Mesh", a8567Mesh)
    REGISTER_SINGLE_FILE_MESH_BENCHMARK("67PESANAVCAMMesh", comet67PMesh)
    REGISTER_SINGLE_FILE_MESH_BENCHMARK("ToutatisMesh", toutatisMesh)
    REGISTER_SINGLE_FILE_MESH_BENCHMARK("ItokawaObjectMesh", itokawaObjectMesh)
    REGISTER_SINGLE_FILE_MESH_BENCHMARK("Hartley2Mesh", hartley2Mesh)
    REGISTER_SINGLE_FILE_MESH_BENCHMARK("ShapeSFMMesh", shapeSfmMesh)
    REGISTER_SINGLE_FILE_MESH_BENCHMARK("MU69Mesh", mu69Mesh)

#undef REGISTER_SINGLE_FILE_MESH_BENCHMARK
}// namespace kdtree

BENCHMARK_MAIN();
