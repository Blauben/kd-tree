#pragma once

#include "KDTree/tree/KDTree.h"
#include "matplot/matplot.h"

#include <string>

namespace kdtree::plotting {
    /**
     * This function generates a 3d plot of the KDTree provided, containing the vertices, if present faces and the split planes.
     * @param tree The instance of KDTree to be plotted.
     * @param outputPath The output path to where the resulting image should be written.
     */
    void plotKDTree(const KDTree &tree, const std::string &outputPath);
}