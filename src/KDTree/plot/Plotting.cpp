#include "KDTree/plot/Plotting.h"

namespace kdtree::plotting {
    void plotKDTree(const KDTree &tree, const std::string &outputPath) {
        auto fig = matplot::plot(std::vector{0, 1}, std::vector{0, 1});
        matplot::save(outputPath);
        //matplot::show();
    }
}