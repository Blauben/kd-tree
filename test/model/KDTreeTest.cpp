#include "KDTree/tree/KDTree.h"
#include "KDTree/input/TetgenAdapter.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <array>
#include <indicators/progress_bar.hpp>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace kdtree {
    using testing::ContainerEq;
    using testing::Contains;
    using testing::DoubleNear;
    using testing::ElementsAre;
    using testing::Pair;
    using Algorithm = PlaneSelectionAlgorithm::Algorithm;

    // Param tuple used by the parameterized test
    using ParamType = std::tuple<std::vector<Vertex>, std::vector<IndexVector>, Algorithm, std::vector<Vertex>>;

    namespace {
        // Test constants
        constexpr long long SEED = 4142561877;
        constexpr double DELTA = 1e-8;
        auto gen = std::mt19937(SEED);

        const std::vector<Vertex> cube_vertices{
                {-1.0, -1.0, -1.0},
                {1.0, -1.0, -1.0},
                {1.0, 1.0, -1.0},
                {-1.0, 1.0, -1.0},
                {-1.0, -1.0, 1.0},
                {1.0, -1.0, 1.0},
                {1.0, 1.0, 1.0},
                {-1.0, 1.0, 1.0}};

        const std::vector<IndexVector> cube_faces{
                {1, 3, 2},
                {0, 3, 1},
                {0, 1, 5},
                {0, 5, 4},
                {0, 7, 3},
                {0, 4, 7},
                {1, 2, 6},
                {1, 6, 5},
                {2, 3, 6},
                {3, 7, 6},
                {4, 5, 6},
                {4, 6, 7}};

        // Lazy load big polyhedron to avoid expensive global initialization at translation time
        const std::tuple<std::vector<Vertex>, std::vector<IndexVector>> &getBigPolyhedron() {
            static const std::vector<std::string> polyhedronNodeFilePath = {
                    std::format("resources/Eros_scaled-{}.node", 27000),
                    std::format("resources/Eros_scaled-{}.face", 27000)};
            static const auto poly = TetgenAdapter{polyhedronNodeFilePath}.getPolyhedralSource();
            return poly;
        }

        int getRandomIndex(const size_t sizeBuffer) {
            std::uniform_int_distribution<> distrib(0, static_cast<int>(sizeBuffer) - 1);
            return distrib(gen);
        }

        Vertex randomPointOnFace(const std::array<Vertex, 3> &vertices) {
            using namespace util;
            std::uniform_real_distribution<> distrib(0.0, 1.0);
            const double a = distrib(gen);
            distrib = std::uniform_real_distribution<>(0.0, 1.0 - a);
            const double b = distrib(gen);
            const double c = 1.0 - a - b;
            return vertices[0] * a + vertices[1] * b + vertices[2] * c;
        }

        std::vector<Vertex> generateRandomPointsOnPolyhedron(const std::vector<Vertex> &vertices,
                                                             const std::vector<IndexVector> &faces,
                                                             const size_t n) {
            std::vector<Vertex> randomPoints;
            randomPoints.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                const auto faceIndex = getRandomIndex(faces.size());
                const auto &verticeIndices = faces.at(faceIndex);
                std::array<Vertex, 3> faceVertices{};
                std::ranges::transform(verticeIndices, faceVertices.begin(),
                                       [&](const auto index) { return vertices.at(index); });
                randomPoints.push_back(randomPointOnFace(faceVertices));
            }
            return randomPoints;
        }

        ObjectIndexVector extractFaceIndices(const std::variant<ObjectIndexVector, PlaneEventVector> &triangles) {
            if (std::holds_alternative<ObjectIndexVector>(triangles)) {
                auto triangleVector = std::get<ObjectIndexVector>(triangles);
                std::ranges::sort(triangleVector);
                return triangleVector;
            }
            std::set<unsigned long> boundTriangles;
            for (const auto &planeEvent: std::get<PlaneEventVector>(triangles)) {
                boundTriangles.insert(planeEvent.objIndex);
            }
            ObjectIndexVector result{boundTriangles.cbegin(), boundTriangles.cend()};
            std::ranges::sort(result);
            return result;
        }

        SplitParam &holdsFaceIndices(SplitParam &param) {
            param.boundObjects = extractFaceIndices(param.boundObjects);
            return param;
        }

        std::pair<ObjectIndexVector, ObjectIndexVector> extractFaceIndicesFromVectors(
                const std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>> &vectors) {
            return std::visit([](const auto &triangleVectors) {
                auto minFaces = extractFaceIndices(*(triangleVectors[0]));
                auto maxFaces = extractFaceIndices(*(triangleVectors[1]));
                return std::make_pair(minFaces, maxFaces);
            },
                              vectors);
        }
    }// namespace

    class KDTreeTest : public ::testing::TestWithParam<ParamType> {};

    TEST_P(KDTreeTest, PointsTest) {
        using namespace util;
        const auto [vertices, faces, algorithm, points] = GetParam();
        indicators::ProgressBar bar{
                indicators::option::BarWidth{50},
                indicators::option::Start{"["},
                indicators::option::End{"]"},
                indicators::option::MaxProgress{points.size()}};
        KDTree tree{vertices, faces, algorithm};
        constexpr Vertex origin{200, 200, 200};
        auto pointTest = [&tree, &origin](const Vertex &point) {
            const auto ray = point - origin;
            std::set<Vertex> intersections;
            tree.getIntersections(origin, ray, intersections);
            ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA),
                                                            DoubleNear(point[1], DELTA),
                                                            DoubleNear(point[2], DELTA))));
        };
        for (const auto &p: points) {
            pointTest(p);
            bar.tick();
        }
    }

    TEST_P(KDTreeTest, ParticleTree) {
        using namespace util;
        constexpr double SPHERE_INTERSECTION_DELTA = 1e-2;
        const auto [vertices, _, algorithm, points] = GetParam();// faces unused here
        indicators::ProgressBar bar{
                indicators::option::BarWidth{50},
                indicators::option::Start{"["},
                indicators::option::End{"]"},
                indicators::option::MaxProgress{points.size()}};
        auto rng = std::mt19937(SEED);
        KDTree tree{vertices, algorithm};
        tree.prebuildTree();
        constexpr Vertex origin{200, 200, 200};
        auto pointTest = [&tree, &origin](const Vertex &point, const size_t pointIndex) {
            const auto ray = point - origin;
            std::set<Vertex> intersections{};
            tree.getIntersections(origin, ray, intersections);
            ASSERT_THAT(intersections,
                        Contains(ElementsAre(DoubleNear(point[0], SPHERE_INTERSECTION_DELTA),
                                             DoubleNear(point[1], SPHERE_INTERSECTION_DELTA),
                                             DoubleNear(point[2], SPHERE_INTERSECTION_DELTA))))
                    << "PointIndex: " << pointIndex;
        };

        for (size_t i = 0; i < points.size(); ++i) {
            const auto index = rng() % vertices.size();
            pointTest(vertices[index], index);
            bar.tick();
        }
    }

    TEST_P(KDTreeTest, AlgorithmRegressionTest) {
        using namespace util;
        std::vector<Vertex> vertices;
        std::vector<IndexVector> faces;
        Algorithm algorithm;
        std::tie(vertices, faces, algorithm, std::ignore) = GetParam();
        KDTree tree{vertices, faces, algorithm};
        auto squaredAlgorithm = PlaneSelectionAlgorithmFactory::create(Algorithm::QUADRATIC);
        auto variantAlgorithm = PlaneSelectionAlgorithmFactory::create(algorithm);

        std::deque<std::shared_ptr<TreeNode>> nodePtrQueue;
        nodePtrQueue.push_back(tree.getRootNode());
        while (!nodePtrQueue.empty()) {
            if (auto splitNodePtr = std::dynamic_pointer_cast<SplitNode>(nodePtrQueue.front())) {
                SplitParam param = *splitNodePtr->_splitParam;
                param.splitDirection = splitNodePtr->_plane.orientation;

                const auto [optimalPlane, optimalCost, optimalTriangles] = squaredAlgorithm->findPlane(holdsFaceIndices(param));
                const auto [variantPlane, variantCost, variantTriangles] = variantAlgorithm->findPlane(param);

                const auto optimalTriangleIndices = extractFaceIndicesFromVectors(std::move(optimalTriangles));
                const auto variantTriangleIndices = extractFaceIndicesFromVectors(std::move(variantTriangles));

                ASSERT_EQ(variantPlane, splitNodePtr->_plane) << "FATAL: test logic faulty";
                EXPECT_EQ(optimalCost, variantCost) << "Plane cost check failed for node with id: " << splitNodePtr->nodeId << "; Algorithm: " << variantPlane << ", Optimal: " << optimalPlane << std::endl;
                ASSERT_EQ(optimalPlane, variantPlane) << "Plane check failed for node with id: " << splitNodePtr->nodeId << "; " << variantPlane << " != " << optimalPlane << std::endl;

                ASSERT_THAT(optimalTriangleIndices.first, ContainerEq(variantTriangleIndices.first))
                        << "Triangle locality check (minFaces) failed for node with id: " << splitNodePtr->nodeId << std::endl
                        << "Plane: " << optimalPlane;
                ASSERT_THAT(optimalTriangleIndices.second, ContainerEq(variantTriangleIndices.second))
                        << "Triangle locality check (maxFaces) failed for node with id: " << splitNodePtr->nodeId << std::endl
                        << "Plane: " << optimalPlane;

                nodePtrQueue.push_back(splitNodePtr->getChildNode(0));
                nodePtrQueue.push_back(splitNodePtr->getChildNode(1));
            }
            nodePtrQueue.pop_front();
        }
    }

    constexpr size_t numberOfPoints = 10;
    constexpr size_t bigNumberOfPoints = 1000;

    // Instantiate tests using lazy-loaded big polyhedron
    INSTANTIATE_TEST_SUITE_P(NoTreePointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::NOTREE,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, numberOfPoints));
                                     }()));

    INSTANTIATE_TEST_SUITE_P(QuadraticPointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::QUADRATIC,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, numberOfPoints));
                                     }()));

    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::LOGSQUARED,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, numberOfPoints));
                                     }()));

    INSTANTIATE_TEST_SUITE_P(LogPointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::LOG,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, numberOfPoints));
                                     }()));

    // Cube-based instantiations
    INSTANTIATE_TEST_SUITE_P(NoTreePointsCube, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         return std::make_tuple(cube_vertices, cube_faces, Algorithm::NOTREE,
                                                                generateRandomPointsOnPolyhedron(cube_vertices, cube_faces, numberOfPoints));
                                     }()));

    // Explicit cube instances
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsCube, KDTreeTest,
                             ::testing::Values(
                                     std::make_tuple(cube_vertices, cube_faces, Algorithm::QUADRATIC,
                                                     generateRandomPointsOnPolyhedron(cube_vertices, cube_faces, numberOfPoints))));

    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsCube, KDTreeTest,
                             ::testing::Values(
                                     std::make_tuple(cube_vertices, cube_faces, Algorithm::LOGSQUARED,
                                                     generateRandomPointsOnPolyhedron(cube_vertices, cube_faces, numberOfPoints))));

    INSTANTIATE_TEST_SUITE_P(LogPointsCube, KDTreeTest,
                             ::testing::Values(
                                     std::make_tuple(cube_vertices, cube_faces, Algorithm::LOG,
                                                     generateRandomPointsOnPolyhedron(cube_vertices, cube_faces, numberOfPoints))));

    // Large point-count instantiations
    INSTANTIATE_TEST_SUITE_P(NoTreeGreatNumberOfPointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::NOTREE,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, bigNumberOfPoints));
                                     }()));

    INSTANTIATE_TEST_SUITE_P(LogSquaredGreatNumberOfPointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::LOGSQUARED,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, bigNumberOfPoints));
                                     }()));

    INSTANTIATE_TEST_SUITE_P(LogGreatNumberOfPointsBig, KDTreeTest,
                             ::testing::Values(
                                     []() -> ParamType {
                                         const auto [vertices, faces] = getBigPolyhedron();
                                         return std::make_tuple(vertices, faces, Algorithm::LOG,
                                                                generateRandomPointsOnPolyhedron(vertices, faces, bigNumberOfPoints));
                                     }()));

}// namespace kdtree
