#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {

    std::ostream &operator<<(std::ostream &os, const Direction &direction) {
        switch (direction) {
            case Direction::X:
                os << "X";
                break;
            case Direction::Y:
                os << "Y";
                break;
            case Direction::Z:
                os << "Z";
                break;
        }
        return os;
    }

    size_t recursionDepth(const size_t nodeId) {
        //t: depth of the current node, N: amount of nodes in the tree if tree is complete
        // $ N = 2^(t+1)-1 $ (geometric series formula for partial sums), assume $ N >= idx + 1 $
        return static_cast<size_t>(std::ceil(std::log2(nodeId + 2))) - 1;
    }
}// namespace kdtree
