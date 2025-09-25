#include "KDTree/tree/KDTree.h"

#include "../../src/KDTree/input/TetgenAdapter.h"
#include "KDTree/input/TetgenAdapter.h"

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

    class KDTreeTest : public ::testing::TestWithParam<std::tuple<PointVector,
                Algorithm>> {
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

    TEST_P(KDTreeTest, BuildTree) {
        using namespace kdtree;
        using namespace util;
        const auto [points, algorithm] = GetParam();
        KDTree tree{points, algorithm};
        ASSERT_NO_THROW(tree.prebuildTree());
        std::cout << tree;
    }

    INSTANTIATE_TEST_SUITE_P(BuildTree, KDTreeTest, ::testing::Values(std::make_tuple(KDTreeTest::cube_points, PlaneSelectionAlgorithm::Algorithm::QUADRATIC)));
} // namespace kdtree
