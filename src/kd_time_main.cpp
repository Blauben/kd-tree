#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"

#include <algorithm>
#include <benchmark/benchmark.h>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
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

        /**
         * If true, basename is a complete path to a single mesh file (e.g. a .ply) and nodeCounts is ignored - exactly one
         * mesh is loaded from it directly. If false, basename is the shared prefix of a series of variants distinguished by
         * their node count (e.g. "resources/Eros_scaled" with nodeCount 1000 -> "resources/Eros_scaled-1000.node"/".face"),
         * one mesh loaded per entry in nodeCounts.
         */
        const bool singleFileMesh;

        Meshes(const std::string &basename, const std::vector<long long> &nodeCounts, const bool singleFileMesh)
            : singleFileMesh{singleFileMesh} {
            if (nodeCounts.empty()) {
                loadMesh(basename, std::nullopt);
            } else {
                std::ranges::for_each(nodeCounts, [this, &basename](const long long nodeCount) {
                    loadMesh(basename, nodeCount);
                });
            }
        }

        std::tuple<const std::vector<Vertex> &, const std::vector<IndexVector> &, const std::vector<Vertex> &> operator[](const size_t index) const {
            return {vertices[index], faces[index], centroids[index]};
        }

        [[nodiscard]] long long size() const {
            return static_cast<long long>(vertices.size());
        }

    private:
        void loadMesh(const std::string &filePath, const std::optional<long long> &nodeCount) {
            const auto [fileVertices, fileFaces] = TetgenAdapter{buildCompleteFilePaths(filePath, nodeCount)}.getPolyhedralSource();
            centroids.push_back(getPolyhedralFaceCentroids(fileVertices, fileFaces));
            vertices.push_back(fileVertices);
            faces.push_back(fileFaces);
        }

        // Builds the file path(s) to load a mesh variant from. filePath is the basename shared across all variants;
        // nodeCount, if present, identifies which variant. For a single mesh file, the count (if any) is spliced in
        // before the extension (e.g. "resources/a8567.tab.ply" + 1000 -> "resources/a8567.tab-1000.ply"); with no count
        // the path is used as-is. For a .node/.face pair, the count (if any) is appended before the extensions are added.
        // PR #60 will introduce scaled .ply variants, so this function will be updated to handle that case as well.
        [[nodiscard]] std::vector<std::string> buildCompleteFilePaths(const std::string &filePath, const std::optional<long long> &nodeCount) const {
            if (singleFileMesh) {
                if (!nodeCount.has_value()) {
                    return {filePath};
                }
                const std::filesystem::path path{filePath};
                return {(path.parent_path() / (path.stem().string() + "-" + std::to_string(*nodeCount) + path.extension().string())).string()};
            }
            const std::string suffixedPath = nodeCount.has_value() ? filePath + "-" + std::to_string(*nodeCount) : filePath;
            return {suffixedPath + ".node", suffixedPath + ".face"};
        }
    };

    const std::vector<long long> scaledMeshNodeCounts{1000, 1732, 3000, 5196, 9000, 15588, 27000, 46765, 81000, 140296};

    Meshes erosMeshes{"resources/Eros_scaled", scaledMeshNodeCounts, false};

    Meshes sphereMeshes{"resources/sphere_scaled", scaledMeshNodeCounts, false};

    Meshes a8567Mesh{"resources/a8567.tab.ply", {}, true};
    Meshes comet67PMesh{"resources/67P_ESA_NAVCAM_Jul2015data_256k.ply", {}, true};
    Meshes toutatisMesh{"resources/4179toutatis.tab.ply", {}, true};
    Meshes itokawaObjectMesh{"resources/Object 25143_Itokawa_200k.ply", {}, true};
    Meshes hartley2Mesh{"resources/hartley2_2012_cart.ply", {}, true};
    Meshes shapeSfmMesh{"resources/SHAPE_SFM_3M_v20180804.ply", {}, true};
    Meshes mu69Mesh{"resources/MU69_Merged.ply", {}, true};

    static __itt_domain *kdTreeIttDomain = __itt_domain_create("KDTree");

    // ITT string handles are meant to be created once per name and cached, not re-created on every benchmark
    // function invocation (each DenseRange entry and --benchmark_repetitions re-run calls the benchmark function
    // again with the same state.name()). Cache them here instead of paying that cost inside the measured process.
    static __itt_string_handle *getIttStringHandle(const std::string &name) {
        static std::unordered_map<std::string, __itt_string_handle *> handles;
        auto [it, inserted] = handles.try_emplace(name, nullptr);
        if (inserted) {
            it->second = __itt_string_handle_create(name.c_str());
        }
        return it->second;
    }

    void BM_Intersection_Tree(benchmark::State &state, const Meshes &mesh, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        auto buildHandle = getIttStringHandle("build_" + state.name());
        auto queryHandle = getIttStringHandle("query_" + state.name());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = mesh[state.range(0)];
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

    void BM_Intersection_Tree_Twice(benchmark::State &state, const Meshes &mesh) {
        auto buildHandle = getIttStringHandle("build_" + state.name());
        auto queryHandle = getIttStringHandle("query_" + state.name());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = mesh[state.range(0)];
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

    void BM_Intersection_Tree_Build(benchmark::State &state, const Meshes &mesh, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        auto buildHandle = getIttStringHandle("build_" + state.name());
        using namespace kdtree::util;
        const auto [vertices, faces, centroids] = mesh[state.range(0)];
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


#define REGISTER_MESH_BENCHMARKS(prefix, mesh)                                                                                                                                                                          \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/NoTree", mesh, PlaneSelectionAlgorithm::Algorithm::NOTREE)->Name(prefix "/NoTree")->DenseRange(0, mesh.size() - 1, 1);                                             \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/Quadratic", mesh, PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Name(prefix "/Quadratic")->DenseRange(0, std::max(mesh.size() - 3, 0LL), 1);                     \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/LogSquared", mesh, PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Name(prefix "/LogSquared")->DenseRange(0, mesh.size() - 1, 1);                                 \
    BENCHMARK_CAPTURE(BM_Intersection_Tree, prefix "/Log", mesh, PlaneSelectionAlgorithm::Algorithm::LOG)->Name(prefix "/Log")->DenseRange(0, mesh.size() - 1, 1);                                                      \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Twice, prefix "/SecondRun", mesh)->Name(prefix "/SecondRun")->DenseRange(0, mesh.size() - 1, 1);                                                                             \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Build, prefix "/BuildTreeSquared", mesh, PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Name(prefix "/BuildTreeSquared")->DenseRange(0, std::max(mesh.size() - 3, 0LL), 1); \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Build, prefix "/BuildTreeLogSquared", mesh, PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Name(prefix "/BuildTreeLogSquared")->DenseRange(0, mesh.size() - 1, 1);         \
    BENCHMARK_CAPTURE(BM_Intersection_Tree_Build, prefix "/BuildTreeLog", mesh, PlaneSelectionAlgorithm::Algorithm::LOG)->Name(prefix "/BuildTreeLog")->DenseRange(0, mesh.size() - 1, 1);

    // eros mesh benchmarks
    REGISTER_MESH_BENCHMARKS("ErosPolyhedron", erosMeshes)

    // sphere mesh benchmarks
    REGISTER_MESH_BENCHMARKS("SpherePolyhedron", sphereMeshes)

    // ply file mesh benchmarks
    REGISTER_MESH_BENCHMARKS("A8567Mesh", a8567Mesh)
    REGISTER_MESH_BENCHMARKS("67PESANAVCAMMesh", comet67PMesh)
    REGISTER_MESH_BENCHMARKS("ToutatisMesh", toutatisMesh)
    REGISTER_MESH_BENCHMARKS("ItokawaObjectMesh", itokawaObjectMesh)
    REGISTER_MESH_BENCHMARKS("Hartley2Mesh", hartley2Mesh)
    REGISTER_MESH_BENCHMARKS("ShapeSFMMesh", shapeSfmMesh)
    REGISTER_MESH_BENCHMARKS("MU69Mesh", mu69Mesh)
#undef REGISTER_MESH_BENCHMARKS

}// namespace kdtree

BENCHMARK_MAIN();
