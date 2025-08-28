#include <vector>
#include "KDTree/tree/KDTree.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"

namespace kdtree {
    int main() {
        std::vector<Array3> vertices{};
        std::vector<IndexArray3> faces{};
        PlaneSelectionAlgorithm::Algorithm algorithm{PlaneSelectionAlgorithm::Algorithm::LOG};

        KDTree tree{vertices, faces, algorithm};
        return 0;
    }
} // namespace kdtree