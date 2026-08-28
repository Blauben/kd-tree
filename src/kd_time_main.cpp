#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"
#include "KDTree/tree/KdDefinitions.h"
#include "ScaledMeshAmounts.h"

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


        Meshes(const std::string &basename, const std::string &mesh_format, const std::optional<std::vector<long long>> &nodeCounts) {
            if (nodeCounts.has_value()) {
                std::ranges::for_each(nodeCounts.value(), [this, &basename, &mesh_format](const long long nodeCount) {
                    loadMesh(basename, mesh_format, nodeCount);
                });
            } else {
                loadMesh(basename, mesh_format, std::nullopt);
            }
        }

        std::tuple<const std::vector<Vertex> &, const std::vector<IndexVector> &, const std::vector<Vertex> &> operator[](const size_t index) const {
            return {vertices[index], faces[index], centroids[index]};
        }

        [[nodiscard]] long long size() const {
            return static_cast<long long>(vertices.size());
        }

    private:
        void loadMesh(const std::string &filePath, const std::string &mesh_format, const std::optional<long long> &nodeCount) {
            const auto [fileVertices, fileFaces] = TetgenAdapter{buildCompleteFilePaths(filePath, nodeCount, mesh_format)}.getPolyhedralSource();
            centroids.push_back(getPolyhedralFaceCentroids(fileVertices, fileFaces));
            vertices.push_back(fileVertices);
            faces.push_back(fileFaces);
        }

        /**
         * This function builds the complete file paths for the mesh files based on the provided base file path, optional node count, and mesh format. If the mesh is a single file mesh, it constructs the file path accordingly. Otherwise, it appends the node count to the base file path and adds the appropriate file extensions for node and face files.
         */
        [[nodiscard]] std::vector<std::string> buildCompleteFilePaths(const std::string &filePath, const std::optional<unsigned> &nodeCount, const std::string &mesh_format) const {
            std::string appendedFilePath = filePath + (nodeCount.has_value() ? "_scaled-" + std::to_string(nodeCount.value()) : "");

            if (mesh_format == "ply") {
                return {appendedFilePath + ".ply"};
            }
            if (mesh_format == "node-face") {
                return {appendedFilePath + ".node", appendedFilePath + ".face"};
            }
            throw std::invalid_argument("Unsupported mesh format: " + mesh_format);
        }
    };

    Meshes erosMeshes{"resources/Eros", "node-face", scaledMeshFaceAmounts};
    Meshes sphereMeshes{"resources/sphere", "node-face", scaledMeshFaceAmounts};
    Meshes a8567Mesh{"resources/a8567.tab", "ply", scaledMeshFaceAmounts};
    Meshes comet67PMesh{"resources/67P_ESA_NAVCAM_Jul2015data_256k", "ply", scaledMeshFaceAmounts};
    Meshes toutatisMesh{"resources/4179toutatis.tab", "ply", scaledMeshFaceAmounts};
    Meshes itokawaObjectMesh{"resources/Object_25143_Itokawa_200k", "ply", scaledMeshFaceAmounts};
    Meshes hartley2Mesh{"resources/hartley2_2012_cart", "ply", scaledMeshFaceAmounts};
    Meshes shapeSfmMesh{"resources/SHAPE_SFM_3M_v20180804", "ply", scaledMeshFaceAmounts};
    Meshes mu69Mesh{"resources/MU69_Merged", "ply", scaledMeshFaceAmounts};

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
