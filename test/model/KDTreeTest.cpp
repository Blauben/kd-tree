#include "KDTree/tree/KDTree.h"

#include "../../src/KDTree/input/TetgenAdapter.h"
#include "KDTree/input/TetgenAdapter.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
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

    using TestData = std::tuple<PointVector,
                Algorithm>;

    class KDTreeTest : public ::testing::TestWithParam<TestData> {
    public:
        static const std::vector<Array3> cube_points;

    protected:
        static constexpr long long SEED = 4142561877;
        static constexpr double DELTA = 1e-8;
    };

    const std::vector<Array3> KDTreeTest::cube_points{
        {-1.0, -1.0, -1.0},
        {1.0, -1.0, -1.0},
        {1.0, 1.0, -1.0},
        {-1.0, 1.0, -1.0},
        {-1.0, -1.0, 1.0},
        {1.0, -1.0, 1.0},
        {1.0, 1.0, 1.0},
        {-1.0, 1.0, 1.0}
    };

    const std::vector<Array3> big_points = []() {
        PointVector points;
        points.reserve(100);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(-10.0, 10.0);

        for (int i = 0; i < 100; ++i) {
            points.push_back({dis(gen), dis(gen), dis(gen)});
        }
        return points;
    }();

    struct TestNameGenerator {
        std::string operator()(const ::testing::TestParamInfo<std::tuple<PointVector, Algorithm>>& info) const {
            const auto& [points, algorithm] = info.param;

            std::string point_set_name;
            if (points.size() == 8) {
                point_set_name = "CubePoints";
            } else if (points.size() == 100) {
                point_set_name = "BigPoints";
            } else {
                point_set_name = "Points" + std::to_string(points.size());
            }

            std::string algorithm_name;
            switch (algorithm) {
                case Algorithm::QUADRATIC:
                    algorithm_name = "Quadratic";
                    break;
                    // Add other algorithm cases as needed
                default:
                    algorithm_name = "Unknown";
                    break;
            }

            return point_set_name + "_" + algorithm_name;
        }
    };

    const std::vector<TestData> testData = {
        std::make_tuple(KDTreeTest::cube_points, Algorithm::QUADRATIC),
        std::make_tuple(big_points, Algorithm::QUADRATIC)
    };

    TEST_P(KDTreeTest, BuildTree) {
        using namespace kdtree;
        using namespace util;
        const auto [points, algorithm] = GetParam();
        KDTree tree{points, algorithm};
        ASSERT_NO_THROW(tree.prebuildTree());
        std::cout << "Points: [";
        std::for_each(points.cbegin(), points.cend(), [](const auto& point) {
            std::cout <<  " {" << point[0] << ", " << point[1] << ", " << point[2] << "} ";
        });
        std::cout << "]" << std::endl << std::endl << "Tree:" << std::endl;
        std::cout << tree << std::endl;
    }

    INSTANTIATE_TEST_SUITE_P(BuildTree, KDTreeTest, ::testing::ValuesIn(testData), TestNameGenerator{});
} // namespace kdtree