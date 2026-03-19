#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "KDTree/tree/KDTree.h"
#include "KDTree/plot/Plotting.h"

namespace kdtree {
    class KDTreePlotTest : public ::testing::Test {};
    TEST_F(KDTreePlotTest, KDTreePlot) {
        const std::string mesh{"resources/Eros_scaled-1000"};
        const KDTree tree{mesh + ".node", mesh + ".face"};
        plotting::plotKDTree(tree, "Eros_scaled-1000_plot.png");
    }
}