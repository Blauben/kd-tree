#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <algorithm>
#include <benchmark/benchmark.h>
#include <string>
#include <vector>

#ifdef ENABLE_ITT_INSTRUMENTATION
#include <ittnotify.h>
#else
// No-op shims for the Intel ITT API calls below, so the benchmark bodies stay instrumented and
// readable without requiring the ittapi dependency unless VTune profiling is enabled.
struct __itt_domain {};
struct __itt_string_handle {};
struct __itt_id {};
static constexpr __itt_id __itt_null{};
inline __itt_domain *__itt_domain_create(const char *) {
    return nullptr;
}
inline __itt_string_handle *__itt_string_handle_create(const char *) {
    return nullptr;
}
inline void __itt_frame_begin_v3(const __itt_domain *, const __itt_id *) {
}
inline void __itt_frame_end_v3(const __itt_domain *, const __itt_id *) {
}
inline void __itt_task_begin(const __itt_domain *, __itt_id, __itt_id, __itt_string_handle *) {
}
inline void __itt_task_end(const __itt_domain *) {
}
#endif

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

    static __itt_domain *kdTreeIttDomain = __itt_domain_create("KDTree");

    void BM_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        auto buildHandle = __itt_string_handle_create(("build_" + state.name()).c_str());
        auto queryHandle = __itt_string_handle_create(("query_" + state.name()).c_str());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        __itt_frame_begin_v3(kdTreeIttDomain, nullptr);
        for (auto _: state) {
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, buildHandle);
            KDTree tree{vertices, faces, algorithm};
            __itt_task_end(kdTreeIttDomain);
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, queryHandle);
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            __itt_task_end(kdTreeIttDomain);
            benchmark::ClobberMemory();
            intersections.clear();
        }
        __itt_frame_end_v3(kdTreeIttDomain, nullptr);
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Intersection_Tree_Twice(benchmark::State &state) {
        auto buildHandle = __itt_string_handle_create(("build_" + state.name()).c_str());
        auto queryHandle = __itt_string_handle_create(("query_" + state.name()).c_str());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        __itt_frame_begin_v3(kdTreeIttDomain, nullptr);
        for (auto _: state) {
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, buildHandle);
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            __itt_task_end(kdTreeIttDomain);
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, queryHandle);
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            __itt_task_end(kdTreeIttDomain);
            benchmark::ClobberMemory();
            intersections.clear();
        }
        __itt_frame_end_v3(kdTreeIttDomain, nullptr);
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        auto buildHandle = __itt_string_handle_create(("build_" + state.name()).c_str());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = erosMeshes[state.range(0)];
        __itt_frame_begin_v3(kdTreeIttDomain, nullptr);
        for (auto _: state) {
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, buildHandle);
            KDTree tree{vertices, faces, algorithm};
            tree.prebuildTree();
            __itt_task_end(kdTreeIttDomain);
            benchmark::ClobberMemory();
        }
        __itt_frame_end_v3(kdTreeIttDomain, nullptr);
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Single_File_Mesh_Intersection_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm, const SingleFileMeshes &meshes) {
        auto buildHandle = __itt_string_handle_create(("build_" + state.name()).c_str());
        auto queryHandle = __itt_string_handle_create(("query_" + state.name()).c_str());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = meshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        __itt_frame_begin_v3(kdTreeIttDomain, nullptr);
        for (auto _: state) {
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, buildHandle);
            KDTree tree{vertices, faces, algorithm};
            __itt_task_end(kdTreeIttDomain);
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, queryHandle);
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            __itt_task_end(kdTreeIttDomain);
            benchmark::ClobberMemory();
            intersections.clear();
        }
        __itt_frame_end_v3(kdTreeIttDomain, nullptr);
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Single_File_Mesh_Intersection_Tree_Twice(benchmark::State &state, const SingleFileMeshes &meshes) {
        auto buildHandle = __itt_string_handle_create(("build_" + state.name()).c_str());
        auto queryHandle = __itt_string_handle_create(("query_" + state.name()).c_str());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = meshes[state.range(0)];
        constexpr Vertex origin{0, 0, 0};
        std::set<Vertex> intersections;
        __itt_frame_begin_v3(kdTreeIttDomain, nullptr);
        for (auto _: state) {
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, buildHandle);
            KDTree tree{vertices, faces};
            tree.prebuildTree();
            __itt_task_end(kdTreeIttDomain);
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, queryHandle);
            std::ranges::for_each(centroids, [&](const Vertex &centroid) {
                tree.getIntersections(origin, (centroid - origin) / 10., intersections);
            });
            __itt_task_end(kdTreeIttDomain);
            benchmark::ClobberMemory();
            intersections.clear();
        }
        __itt_frame_end_v3(kdTreeIttDomain, nullptr);
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }

    void BM_Single_File_Mesh_Intersection_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm, const SingleFileMeshes &meshes) {
        auto buildHandle = __itt_string_handle_create(("build_" + state.name()).c_str());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = meshes[state.range(0)];
        __itt_frame_begin_v3(kdTreeIttDomain, nullptr);
        for (auto _: state) {
            __itt_task_begin(kdTreeIttDomain, __itt_null, __itt_null, buildHandle);
            KDTree tree{vertices, faces, algorithm};
            tree.prebuildTree();
            __itt_task_end(kdTreeIttDomain);
            benchmark::ClobberMemory();
        }
        __itt_frame_end_v3(kdTreeIttDomain, nullptr);
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(faces.size()));
    }
    // eros mesh benchmarks
#define REGISTER_POLYHEDRON_MESH_BENCHMARKS(prefix, mesh)                                                                                                                                                 \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/NoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->Name(prefix "/NoTree")->DenseRange(0, mesh.size() - 1, 1);                                     \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/Quadratic", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Name(prefix "/Quadratic")->DenseRange(0, mesh.size() - 3, 1);                            \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/LogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Name(prefix "/LogSquared")->DenseRange(0, mesh.size() - 1, 1);                         \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/Log", PlaneSelectionAlgorithm::Algorithm::LOG)->Name(prefix "/Log")->DenseRange(0, mesh.size() - 1, 1);                                              \
    BENCHMARK(BM_Intersection_Tree_Twice)->Name(prefix "/SecondRun")->DenseRange(0, mesh.size() - 1, 1);                                                                                                  \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Build, prefix "/BuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Name(prefix "/BuildTreeSquared")->DenseRange(0, mesh.size() - 3, 1);        \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Build, prefix "/BuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Name(prefix "/BuildTreeLogSquared")->DenseRange(0, mesh.size() - 1, 1); \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Build, prefix "/BuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->Name(prefix "/BuildTreeLog")->DenseRange(0, mesh.size() - 1, 1);

    REGISTER_POLYHEDRON_MESH_BENCHMARKS("ErosPolyhedron", erosMeshes)

    // sphere mesh benchmarks
    REGISTER_POLYHEDRON_MESH_BENCHMARKS("SpherePolyhedron", sphereMeshes)

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
