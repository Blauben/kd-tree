#include <iostream>
#include <vector>
#include "KDTree/tree/KDTree.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"

int main() {
    using namespace kdtree;
    std::vector<Array3> vertices{};
    std::vector<IndexArray3> faces{};
    PlaneSelectionAlgorithm::Algorithm algorithm{PlaneSelectionAlgorithm::Algorithm::LOG};

    KDTree tree{vertices, faces, algorithm};
    std::cout << tree;
    return 0;
}