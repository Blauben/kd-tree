#include "KDTree/tree/KDTree.h"
#include "KDTree/input/TetgenAdapter.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <array>
#include <cstdlib>
#include <indicators/progress_bar.hpp>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <streambuf>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace kdtree {
    using testing::Contains;
    using testing::DoubleNear;
    using testing::ElementsAre;
    using testing::Pair;
    using testing::UnorderedElementsAreArray;
    using Algorithm = PlaneSelectionAlgorithm::Algorithm;

    // Param tuple used by the parameterized test
    using ParamType = std::tuple<std::vector<std::array<double, 3>>, std::vector<IndexVector>, Algorithm, std::vector<std::array<double, 3>>>;

    namespace {
        /**
         * Test constants
        */
        constexpr long long SEED = 4142561877;
        /**
         * A small delta value for comparing floating-point numbers in tests
         */
        constexpr double DELTA = 1e-8;
        /**
         * seed for random number generation to ensure reproducibility in tests
         */
        auto gen = std::mt19937(SEED);

        /**
         * Discards everything written to it; used to silence indicators::ProgressBar output on CI,
         * where its carriage-return-based redraws would otherwise flood the log with noise.
         */
        class NullBuffer final : public std::streambuf {
        protected:
            int overflow(int character) override {
                return character;
            }
        };

        /**
         * Returns the stream indicators::ProgressBar should render to: stdout normally, or a
         * discarding stream when the CI env var is set, to keep CI logs readable.
         */
        [[nodiscard]] std::ostream &progressBarStream() {
            static NullBuffer nullBuffer;
            static std::ostream nullStream{&nullBuffer};
            return std::getenv("CI") != nullptr ? nullStream : std::cout;
        }

        /**
         * Computes the squared Euclidean distance between two vertices.
         *
         * The squared form is sufficient for nearest-point comparisons and avoids
         * the extra cost and numeric noise of taking square roots.
         *
         * @param lhs First vertex.
         * @param rhs Second vertex.
         * @return Squared Euclidean distance between @p lhs and @p rhs.
         */
        [[nodiscard]] double squaredDistance(const std::array<double, 3> &lhs, const std::array<double, 3> &rhs) {
            const double dx = lhs[0] - rhs[0];
            const double dy = lhs[1] - rhs[1];
            const double dz = lhs[2] - rhs[2];
            return dx * dx + dy * dy + dz * dz;
        }

        /**
         * Formats a vertex as a compact string for test diagnostics.
         * @param vertex Vertex to format.
         * @return String in the form [x, y, z].
         */
        [[nodiscard]] std::string vertexToString(const std::array<double, 3> &vertex) {
            std::ostringstream stream;
            stream << "[" << vertex[0] << ", " << vertex[1] << ", " << vertex[2] << "]";
            return stream.str();
        }

        /**
         * Builds a human-readable assertion message with nearest-intersection details.
         *
         * If no intersections are found, the message explicitly reports that the set is empty.
         * Otherwise, it reports the expected point, closest returned point, squared distance,
         * and number of intersections.
         *
         * @param intersections Intersections returned by the KDTree query.
         * @param expectedPoint Target point that should have been intersected.
         * @return Diagnostic message appended to gtest failure output.
         */
        [[nodiscard]] std::string closestIntersectionMessage(const std::set<std::array<double, 3>> &intersections, const std::array<double, 3> &expectedPoint) {
            if (intersections.empty()) {
                return "Closest intersection: <none> (intersection set is empty)";
            }

            const auto closest = std::min_element(intersections.begin(), intersections.end(), [&expectedPoint](const std::array<double, 3> &lhs, const std::array<double, 3> &rhs) {
                return squaredDistance(lhs, expectedPoint) < squaredDistance(rhs, expectedPoint);
            });

            std::ostringstream stream;
            stream << "Expected point: " << vertexToString(expectedPoint)
                   << ", closest intersection: " << vertexToString(*closest)
                   << ", squared distance: " << squaredDistance(*closest, expectedPoint)
                   << ", total intersections: " << intersections.size();
            return stream.str();
        }

        /**
         * A simple cube polyhedron for testing purposes: vertices
         */
        const std::vector<std::array<double, 3>> cube_vertices{
                {-1.0, -1.0, -1.0},
                {1.0, -1.0, -1.0},
                {1.0, 1.0, -1.0},
                {-1.0, 1.0, -1.0},
                {-1.0, -1.0, 1.0},
                {1.0, -1.0, 1.0},
                {1.0, 1.0, 1.0},
                {-1.0, 1.0, 1.0}};

        /**
         * A simple cube polyhedron for testing purposes: faces defined by vertex indices
         */
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

        /**
         * The (vertices, faces) source data of a loaded polyhedron mesh.
         */
        using PolyhedronSource = std::tuple<std::vector<std::array<double, 3>>, std::vector<IndexVector>>;

        /**
         * Lazy loads and caches a two-file (.node/.face) polyhedron mesh to avoid expensive global
         * initialization at translation time and redundant reloading across test instantiations that
         * reference the same mesh. The files should be located in the resources directory and named
         * "<meshName>-<size>.node" and "<meshName>-<size>.face".
         * @param meshName Base name of the mesh, e.g. "Eros" for "Eros_scaled-27000.node" and "Eros_scaled-27000.face".
         * @param size Size suffix used in the mesh's file names, e.g. 27000 for "Eros_scaled-27000".
         * @return A tuple containing the vertices and faces of the polyhedron.
         */
        const PolyhedronSource &getNodePolyhedron(const std::string &meshName, const size_t size) {
            static std::map<std::string, PolyhedronSource> cache;
            const std::string key = std::format("{}_scaled-{}", meshName, size);

            // Check if the mesh is already cached; if so, return it. Otherwise, load it from the files and cache it.
            auto it = cache.find(key);
            if (it != cache.end()) {
                return it->second;
            }
            const std::vector<std::string> filePaths = {
                    std::format("resources/{}.node", key),
                    std::format("resources/{}.face", key)};
            PolyhedronSource nodeSource = TetgenAdapter{filePaths}.getPolyhedralSource();
            cache[key] = nodeSource;
            return cache[key];
        }

        /**
         * Lazy loads and caches a single-file (.ply) polyhedron mesh to avoid expensive global
         * initialization at translation time and redundant reloading across test instantiations that
         * reference the same mesh. The file should be located in the resources directory.
         * @param fileName Base name of the .ply file, excluding the extension, e.g. "a8567.tab".
         * @param size Size suffix used in the mesh's file names, e.g. 27000 for "a8567.tab_scaled-27000.ply".
         * @return A tuple containing the vertices and faces of the polyhedron.
         */
        const PolyhedronSource &getPlyPolyhedron(const std::string &fileName, const size_t size) {
            static std::map<std::string, PolyhedronSource> cache;
            auto [entry, inserted] = cache.try_emplace(fileName);
            if (inserted) {
                entry->second = TetgenAdapter{{"resources/" + fileName}}.getPolyhedralSource();
            }
            return entry->second;
        }

        /**
         * Generates random point index on the surface of a polyhedron (constraint imposed by specifying the index range through size buffer) defined by its   vertices and faces. Used for testing ray intersections with the KDTree.
         * @param sizeBuffer The size of the index range to generate the random index from.
         * @return A random point index on the surface of the polyhedron.
         */
        int getRandomIndex(const size_t sizeBuffer) {
            std::uniform_int_distribution<> distrib(0, static_cast<int>(sizeBuffer) - 1);
            return distrib(gen);
        }

        /**
         * Generates a random point on the surface of a triangle defined by its vertices. Used for testing ray intersections with the KDTree.
         * @param vertices The vertices of the triangle.
         * @return A random point on the surface of the triangle.
         */
        std::array<double, 3> randomPointOnFace(const std::array<std::array<double, 3>, 3> &vertices) {
            using namespace util;
            std::uniform_real_distribution<> distrib(0.0, 1.0);
            const double a = distrib(gen);
            distrib = std::uniform_real_distribution<>(0.0, 1.0 - a);
            const double b = distrib(gen);
            const double c = 1.0 - a - b;
            return vertices[0] * a + vertices[1] * b + vertices[2] * c;
        }

        /**
         * Generates a vector of random points on the surface of a polyhedron defined by its vertices and faces.
         * @param vertices The vertices of the polyhedron.
         * @param faces The faces of the polyhedron defined by vertex indices.
         * @param n The number of random points to generate.
         * @return A vector of random points on the surface of the polyhedron.
         */
        std::vector<std::array<double, 3>> generateRandomPointsOnPolyhedron(const std::vector<std::array<double, 3>> &vertices,
                                                                            const std::vector<IndexVector> &faces,
                                                                            const size_t n) {
            std::vector<std::array<double, 3>> randomPoints;
            randomPoints.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                const auto faceIndex = getRandomIndex(faces.size());
                const auto &verticeIndices = faces.at(faceIndex);
                std::array<std::array<double, 3>, 3> faceVertices{};
                std::ranges::transform(verticeIndices, faceVertices.begin(),
                                       [&](const auto index) { return vertices.at(index); });
                randomPoints.push_back(randomPointOnFace(faceVertices));
            }
            return randomPoints;
        }

        /**
         * Extracts the indices of the shapes that are referenced by the given PlaneEvents. Used for testing purposes to compare the plane selection algorithms.
         * @param triangles The PlaneEvents containing information about the shapes.
         * @return A list of shape indices.
         */
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

        /**
         * Extracts the indices of the shapes that are referenced by the given PlaneEvents and updates the boundObjects field of the given SplitParam. Used for testing purposes to compare the plane selection algorithms.
          * @param param The SplitParam containing the PlaneEvents in its boundObjects field.
          * @return The updated SplitParam with the extracted shape indices in its boundObjects field.
         */
        SplitParam &holdsFaceIndices(SplitParam &param) {
            param.boundObjects = extractFaceIndices(param.boundObjects);
            return param;
        }

        /**
         * Extracts the face indices from a pair of vectors of face indices.
         * @param vectors The pair of vectors of face indices.
         * @return A pair of vectors of face indices.
         */
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

    /**
     * @class KDTreeTest
     * @brief Test suite for the KDTree class functionality.
     */
    class KDTreeTest : public ::testing::TestWithParam<ParamType> {};

    /**
     * Tests the intersection of rays with points on the surface of a polyhedron using the KDTree. The test generates random points on the surface of the polyhedron and checks if the KDTree correctly identifies the intersection points when rays are cast from a fixed origin towards these points. The test uses a progress bar to track the progress of testing multiple points.
     */
    TEST_P(KDTreeTest, PointsTest) {
        using namespace util;
        auto [vertices, faces, algorithm, points] = GetParam();
        indicators::ProgressBar bar{
                indicators::option::BarWidth{50},
                indicators::option::Start{"["},
                indicators::option::End{"]"},
                indicators::option::MaxProgress{points.size()},
                indicators::option::Stream{progressBarStream()}};
        KDTree tree{vertices, faces, algorithm};
        constexpr Vertex origin{200, 200, 200};
        auto pointTest = [&tree, &origin](const Vertex &point) {
            const auto ray = point - origin;
            std::set<Vertex> intersections;
            tree.getIntersections(origin, ray, intersections);
            ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA),
                                                            DoubleNear(point[1], DELTA),
                                                            DoubleNear(point[2], DELTA))))
                    << closestIntersectionMessage(intersections, point);
        };
        for (const auto &p: points) {
            pointTest(p);
            bar.tick();
        }
    }

    /**
     * This test is similar to the PointsTest but only builds a KDTree with the vertices of the polyhedron and no faces (particles). This means that the tree will not contain any shapes and the test checks if the KDTree can identify the nearest point on a ray to a targeted vertex given a DELTA tolerance.
     */
    TEST_P(KDTreeTest, ParticleTree) {
        using namespace util;
        constexpr double SPHERE_INTERSECTION_DELTA = 1e-2;
        auto [vertices, _, algorithm, points] = GetParam();// faces unused here
        indicators::ProgressBar bar{
                indicators::option::BarWidth{50},
                indicators::option::Start{"["},
                indicators::option::End{"]"},
                indicators::option::MaxProgress{points.size()},
                indicators::option::Stream{progressBarStream()}};
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

    /**
     * Tests the regression of the plane selection algorithms used in the KDTree. The test builds a KDTree using a specified plane selection algorithm and then traverses the tree to compare the planes selected by the algorithm with those selected by a reference algorithm. The test checks if the planes, their costs, and the locality of the triangles they reference are consistent between the two algorithms for each split node in the tree.
     */
    TEST_P(KDTreeTest, AlgorithmRegressionTest) {
        using namespace util;
        constexpr Algorithm referenceAlgorithmType = Algorithm::LOG;
        std::vector<Vertex> vertices;
        std::vector<IndexVector> faces;
        Algorithm algorithm;
        std::tie(vertices, faces, algorithm, std::ignore) = GetParam();
        if (algorithm == referenceAlgorithmType || algorithm == Algorithm::NOTREE) {
            GTEST_SKIP() << "Skipping regression test for notree and the designated reference algorithm as it is used as the reference algorithm.";
        }
        KDTree tree{vertices, faces, referenceAlgorithmType};
        auto referenceAlgorithm = PlaneSelectionAlgorithmFactory::create(referenceAlgorithmType);
        auto variantAlgorithm = PlaneSelectionAlgorithmFactory::create(algorithm);

        std::deque<std::shared_ptr<TreeNode>> nodePtrQueue;
        nodePtrQueue.push_back(tree.getRootNode());
        while (!nodePtrQueue.empty()) {
            if (auto splitNodePtr = std::dynamic_pointer_cast<SplitNode>(nodePtrQueue.front())) {
                SplitParam param = *splitNodePtr->_splitParam;

                const auto [optimalPlane, optimalCost, optimalTriangles] = referenceAlgorithm->findPlane(holdsFaceIndices(param));
                // set the split direction to the optimal plane orientation for the variant algorithm. Some algorithms use round-robin to determine the split direction, which may not match the optimal plane orientation. This ensures that the variant algorithm is tested with the same split direction as the reference algorithm.
                param.splitDirection = optimalPlane.orientation;
                const auto [variantPlane, variantCost, variantTriangles] = variantAlgorithm->findPlane(param);

                const auto optimalTriangleIndices = extractFaceIndicesFromVectors(std::move(optimalTriangles));
                const auto variantTriangleIndices = extractFaceIndicesFromVectors(std::move(variantTriangles));

                ASSERT_EQ(variantPlane, splitNodePtr->_plane) << "FATAL: test logic faulty";
                EXPECT_EQ(optimalCost, variantCost) << "Plane cost check failed for node with id: " << splitNodePtr->nodeId << "; Algorithm: " << variantPlane << ", Optimal: " << optimalPlane << std::endl;
                ASSERT_EQ(optimalPlane, variantPlane) << "Plane check failed for node with id: " << splitNodePtr->nodeId << "; " << variantPlane << " != " << optimalPlane << std::endl;

                ASSERT_THAT(optimalTriangleIndices.first, UnorderedElementsAreArray(variantTriangleIndices.first))
                        << "Triangle locality check (minFaces) failed for node with id: " << splitNodePtr->nodeId << std::endl
                        << "Plane: " << optimalPlane;
                ASSERT_THAT(optimalTriangleIndices.second, UnorderedElementsAreArray(variantTriangleIndices.second))
                        << "Triangle locality check (maxFaces) failed for node with id: " << splitNodePtr->nodeId << std::endl
                        << "Plane: " << optimalPlane;

                nodePtrQueue.push_back(splitNodePtr->getChildNode(0));
                nodePtrQueue.push_back(splitNodePtr->getChildNode(1));
            }
            nodePtrQueue.pop_front();
        }
    }

    namespace {
        /**
         * Recursively counts the particles contained in the subtree rooted at the given node.
         * @param node Root of the subtree to count particles in.
         * @return the number of particles contained in the subtree.
         */
        size_t countParticlesInSubtree(const std::shared_ptr<TreeNode> &node) {
            if (const auto leaf = std::dynamic_pointer_cast<LeafNode>(node)) {
                return leaf->getContainedParticles().size();
            }
            const auto split = std::dynamic_pointer_cast<SplitNode>(node);
            return countParticlesInSubtree(split->getChildNode(0)) + countParticlesInSubtree(split->getChildNode(1));
        }

        /**
         * Recursively counts the SplitNodes in the subtree rooted at the given node by directly walking the tree,
         * independently of PlaneIterator. Used as a ground truth for how many planes PlaneIterator should visit,
         * so the expectation tracks however the plane selection algorithm happens to shape the tree.
         * @param node Root of the subtree to count SplitNodes in.
         * @return the number of SplitNodes contained in the subtree.
         */
        size_t countSplitNodes(const std::shared_ptr<TreeNode> &node) {
            const auto split = std::dynamic_pointer_cast<SplitNode>(node);
            if (!split) {
                return 0;
            }
            return 1 + countSplitNodes(split->getChildNode(0)) + countSplitNodes(split->getChildNode(1));
        }
    }// namespace

    /**
     * Regression test for the particle-mode SAH cost function (see PlaneSelectionAlgorithm::costForPlane): the
     * "empty space" cost discount must not apply when particleMode is set. A zero-extent particle sitting exactly
     * on a candidate split plane is counted as "planar" rather than belonging to either child box, so it is free to
     * be assigned to whichever side is cheaper; once assigned, that side's bound is only tightened to the plane
     * itself, not to the true extent of its remaining particles (Box::splitBox just clips to the plane coordinate).
     * That leaves slack between the box bound and the actual outermost remaining particle, so the very next split
     * can place a plane exactly at that particle's true position and get an "empty" (shapesMin/shapesMax == 0) box
     * for free, entirely regardless of how many particles are actually left. Without the particleMode guard, this
     * discount made the SAH prefer chipping off one boundary particle at a time over an even split, snowballing into
     * a heavily skewed, linked-list-like tree. The particle set below reproduces this: particles spread out along x
     * (so there is always more than one to chip away) while y and z span a much wider, fixed range, which elongates
     * every node's box as x keeps shrinking and makes a real, evenly-dividing split look expensive in surface-area
     * terms compared to a thin "shave one particle off the edge" cut, unless the discount is correctly disabled.
     */
    TEST_F(KDTreeTest, ParticleModeDoesNotIsolateSingleParticles) {
        std::vector<Vertex> particles{};
        constexpr int numberOfParticles = 30;
        for (int i = 0; i < numberOfParticles; ++i) {
            const double wideCoordinate = i % 2 == 0 ? 0.0 : 500.0;
            particles.push_back(Vertex{static_cast<double>(i), wideCoordinate, wideCoordinate});
        }

        KDTree tree{particles, Algorithm::LOG};
        tree.prebuildTree();

        std::deque<std::shared_ptr<TreeNode>> nodeQueue;
        nodeQueue.push_back(tree.getRootNode());
        while (!nodeQueue.empty()) {
            if (const auto splitNode = std::dynamic_pointer_cast<SplitNode>(nodeQueue.front())) {
                const auto lesserCount = countParticlesInSubtree(splitNode->getChildNode(0));
                const auto greaterCount = countParticlesInSubtree(splitNode->getChildNode(1));
                // A split isolating a single particle while more than a handful remain is the signature of the
                // empty-space discount being wrongly applied; a real spatial split should divide particles more evenly.
                if (lesserCount + greaterCount > 3) {
                    EXPECT_GT(std::min(lesserCount, greaterCount), 1)
                            << "Split isolated a single particle instead of dividing the "
                            << (lesserCount + greaterCount) << " particles evenly.";
                }
                nodeQueue.push_back(splitNode->getChildNode(0));
                nodeQueue.push_back(splitNode->getChildNode(1));
            }
            nodeQueue.pop_front();
        }
    }

    /**
     * Tests that triggering a rebuild actually resets the KDTree.
     */
    TEST_F(KDTreeTest, RebuildTreeTest) {
        KDTree tree{cube_vertices, cube_faces, Algorithm::NOTREE};
        ASSERT_THAT(tree.getRootNode(), testing::NotNull());
        tree.rebuildTree();
        ASSERT_THAT(tree._rootNode, testing::IsNull());
        ASSERT_THAT(tree.getRootNode(), testing::NotNull());
    }

    /**
     * Tests the functionality of the plane iterator of the KDTree. The test builds a KDTree from a specified mesh and then uses the plane iterator to traverse the tree and print out the planes and their corresponding bounding boxes. The test checks that the iterator visits every SplitNode in the tree exactly once by comparing the iteration count against an independent tree walk (see countSplitNodes), rather than a hardcoded plane count, since the exact tree shape depends on plane selection/cost constants that are expected to be tuned over time.
     */
    TEST_F(KDTreeTest, IteratorTest) {
        const std::string meshPath = "resources/Eros_scaled-1000";
        KDTree tree{meshPath + ".node", meshPath + ".face"};
        const size_t expectedPlaneCount = countSplitNodes(tree.getRootNode());
        ASSERT_GT(expectedPlaneCount, 0) << "Test tree did not produce any SplitNodes to iterate over!";
        auto [begin, end] = tree.planeIterator();
        unsigned long iteration = 0;
        std::for_each(begin, end, [&](const auto &entry) {
            const auto &[plane, box] = entry;
            iteration++;
            std::cout << "Plane: " << plane << ", Box: " << box << std::endl;
        });
        ASSERT_EQ(iteration, expectedPlaneCount) << "Plane iterator did not iterate over all SplitNodes in the tree!";
    };

    /**
     * Tests that the LeafNode class correctly registers and deregisters leaf nodes in the static leafNodes vector. The test builds a KDTree from a specified mesh and checks that the leafNodes vector is populated after prebuilding the tree and is empty after rebuilding the tree, which should reset the KDTree and clear all registered leaf nodes.
     */
    TEST_F(KDTreeTest, LeafNodeRegisterTest) {
        const std::string meshPath{"resources/Eros_scaled-1000"};
        KDTree tree{meshPath + ".node", meshPath + ".face"};
        ASSERT_TRUE(tree.nodeRegister.leafNodes.empty());
        tree.prebuildTree();
        ASSERT_FALSE(tree.nodeRegister.leafNodes.empty());
        tree.rebuildTree();
        ASSERT_TRUE(tree.nodeRegister.leafNodes.empty());
    }

    /**
     * Tests that modifying vertices of an existing KDTree and triggering a rebuild correctly updates the planes in the tree. The test builds a KDTree from a specified mesh, stores the planes generated by the plane iterator, modifies the vertices by applying a shift, triggers a rebuild of the tree, and then uses the plane iterator again to check if the planes have been updated according to the applied shift. The test asserts that the planes' axis coordinates have been shifted by the expected amount and that the number of planes remains the same after the rebuild.
     */
    TEST_F(KDTreeTest, DynamicVerticesTreeRebuildTest) {
        using namespace util;
        constexpr double shift = 1.0;
        std::string meshPath{"resources/Eros_scaled-1000"};
        auto [vertices, faces] = TetgenAdapter{{meshPath + ".node", meshPath + ".face"}}.getPolyhedralSource();
        std::vector<Vertex> vertices_copy{vertices.begin(), vertices.end()};
        KDTree tree{vertices_copy, faces, Algorithm::LOG, false};
        std::vector<Plane> planesBefore{};
        auto [begin, end] = tree.planeIterator();
        std::for_each(begin, end, [&](const auto &entry) {
            planesBefore.push_back(std::get<0>(entry));
        });
        ASSERT_GT(planesBefore.size(), 0) << "Plane iterator did not iterate over any planes before tree rebuild!";
        for (auto &vertex: vertices_copy) {
            vertex = vertex + std::array{shift, shift, shift};
        }
        tree.rebuildTree();
        auto [beginAfter, endAfter] = tree.planeIterator();
        size_t i = 0;
        for (; beginAfter != endAfter; ++beginAfter) {
            ASSERT_THAT(std::get<0>(*beginAfter).axisCoordinate, DoubleNear(planesBefore[i].axisCoordinate + shift, DELTA)) << "Plane origin point check failed for plane " << i << " after tree rebuild!";
            i++;
        }
        ASSERT_EQ(i, planesBefore.size()) << "Plane iterator did not iterate over the same number of planes after tree rebuild!";
    }

    constexpr size_t numberOfPoints = 10;
    constexpr size_t bigNumberOfPoints = 1000;

    // the shared size suffix used to build big instantiations.
    constexpr size_t SMALL_POLYHEDRON_SIZE = 1000;
    constexpr size_t BIG_POLYHEDRON_SIZE = 27000;

    // Names of the meshes, also used by the benchmark suite.
    constexpr auto EROS_MESH_NAME = "Eros";
    constexpr auto SPHERE_MESH_NAME = "sphere";
    constexpr auto A8567_MESH_FILE = "a8567.tab.ply";
    constexpr auto COMET_67P_MESH_FILE = "67P_ESA_NAVCAM_Jul2015data_256k.ply";
    constexpr auto TOUTATIS_MESH_FILE = "4179toutatis.tab.ply";
    constexpr auto ITOKAWA_MESH_FILE = "Object_25143_Itokawa_200k.ply";
    constexpr auto HARTLEY2_MESH_FILE = "hartley2_2012_cart.ply";
    constexpr auto SHAPE_SFM_MESH_FILE = "SHAPE_SFM_3M_v20180804.ply";
    constexpr auto MU69_MESH_FILE = "MU69_Merged.ply";

    /**
     * Instantiates a single KDTreeTest suite for one plane selection algorithm.
     * @param suiteName Suffix appended after the algorithm label to form the instantiation name (e.g. "PointsBig" -> "LogPointsBig").
     * @param label Human-readable algorithm label used to build the instantiation name (e.g. NoTree, Quadratic, LogSquared, Log).
     * @param algoEnum The PlaneSelectionAlgorithm::Algorithm enumerator to test (e.g. NOTREE, QUADRATIC, LOGSQUARED, LOG).
     * @param polyhedronExpr Expression yielding a (vertices, faces) tuple/pair, e.g. a call to getNodePolyhedron/getPlyPolyhedron.
     * @param pointCount Number of random surface points to generate for the suite.
     */
#define KDTREE_INSTANTIATE_ONE(suiteName, label, algoEnum, polyhedronExpr, pointCount)                                          \
    INSTANTIATE_TEST_SUITE_P(label##suiteName, KDTreeTest,                                                                      \
                             ::testing::Values(                                                                                 \
                                     []() -> ParamType {                                                                        \
                                         const auto [vertices, faces] = polyhedronExpr;                                         \
                                         return std::make_tuple(vertices, faces, Algorithm::algoEnum,                           \
                                                                generateRandomPointsOnPolyhedron(vertices, faces, pointCount)); \
                                     }()))


#define INSTANTIATE_KDTREE_MESH_TESTS(suiteName, polyhedronExpr, pointCount)               \
    KDTREE_INSTANTIATE_ONE(suiteName, NoTree, NOTREE, polyhedronExpr, pointCount);         \
    KDTREE_INSTANTIATE_ONE(suiteName, LogSquared, LOGSQUARED, polyhedronExpr, pointCount); \
    KDTREE_INSTANTIATE_ONE(suiteName, Log, LOG, polyhedronExpr, pointCount)

    // Same as INSTANTIATE_KDTREE_MESH_TESTS but also covers Algorithm::QUADRATIC. Quadratic plane
    // selection is O(faces^2) per split, so this is only used for the small cube mesh; the big
    // polyhedron and .ply meshes below would make it prohibitively slow.
#define INSTANTIATE_KDTREE_MESH_TESTS_WITH_QUADRATIC(suiteName, polyhedronExpr, pointCount) \
    INSTANTIATE_KDTREE_MESH_TESTS(suiteName, polyhedronExpr, pointCount);                   \
    KDTREE_INSTANTIATE_ONE(suiteName, Quadratic, QUADRATIC, polyhedronExpr, pointCount)


    // Eros small polyhedron instantiations
    INSTANTIATE_KDTREE_MESH_TESTS_WITH_QUADRATIC(PointsErosSmall, getNodePolyhedron(EROS_MESH_NAME, SMALL_POLYHEDRON_SIZE), numberOfPoints);

    // Cube-based instantiations
    INSTANTIATE_KDTREE_MESH_TESTS_WITH_QUADRATIC(PointsCube, std::tie(cube_vertices, cube_faces), numberOfPoints);

    // Large point-count instantiations
    INSTANTIATE_KDTREE_MESH_TESTS(PointsErosBig, getNodePolyhedron(EROS_MESH_NAME, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);

    // Sphere-based instantiations
    INSTANTIATE_KDTREE_MESH_TESTS(PointsSphere, getNodePolyhedron(SPHERE_MESH_NAME, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);

    // Single-file (.ply)
    INSTANTIATE_KDTREE_MESH_TESTS(PointsA8567, getPlyPolyhedron(A8567_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);
    INSTANTIATE_KDTREE_MESH_TESTS(PointsComet67P, getPlyPolyhedron(COMET_67P_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);
    INSTANTIATE_KDTREE_MESH_TESTS(PointsToutatis, getPlyPolyhedron(TOUTATIS_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);
    INSTANTIATE_KDTREE_MESH_TESTS(PointsItokawa, getPlyPolyhedron(ITOKAWA_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);
    INSTANTIATE_KDTREE_MESH_TESTS(PointsHartley2, getPlyPolyhedron(HARTLEY2_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);
    INSTANTIATE_KDTREE_MESH_TESTS(PointsShapeSfm, getPlyPolyhedron(SHAPE_SFM_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);
    INSTANTIATE_KDTREE_MESH_TESTS(PointsMu69, getPlyPolyhedron(MU69_MESH_FILE, BIG_POLYHEDRON_SIZE), bigNumberOfPoints);

#undef INSTANTIATE_KDTREE_MESH_TESTS_WITH_QUADRATIC
#undef INSTANTIATE_KDTREE_MESH_TESTS
#undef KDTREE_INSTANTIATE_ONE

}// namespace kdtree
