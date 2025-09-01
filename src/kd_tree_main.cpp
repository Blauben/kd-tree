#include "KDTree/model/TetgenAdapter.h"


#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/tree/KDTree.h"
#include <iostream>
#include <vector>

int main() {
    using namespace kdtree;
    PlaneSelectionAlgorithm::Algorithm algorithm{PlaneSelectionAlgorithm::Algorithm::LOG};

    KDTree tree{"..\\polyhedral_files\\Eros_scaled-140296.node", "..\\polyhedral_files\\Eros_scaled-140296.face", algorithm};
    tree.prebuildTree();
    std::cout << tree;
    return 0;
}