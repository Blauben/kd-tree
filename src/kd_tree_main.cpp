#include "KDTree/Logging.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"
#include <iostream>
#include <vector>

int main() {
    using namespace kdtree;
    PlaneSelectionAlgorithm::Algorithm algorithm{PlaneSelectionAlgorithm::Algorithm::LOG};

    INFO("Starting KD-Tree construction using algorithm: " + std::to_string(static_cast<int>(algorithm)));

    KDTree tree{"../polyhedral_files/Eros_scaled-140296.node", "../polyhedral_files/Eros_scaled-140296.face", algorithm};
    tree.prebuildTree();
    INFO("KD-Tree construction completed.");
    DEBUG("KD-Tree details:\n" + to_string(tree));
    return 0;
}