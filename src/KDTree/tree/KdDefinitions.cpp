#include "KDTree/tree/KdDefinitions.h"

namespace kdtree {

    /**
     * Overloads the output stream operator for Direction enum class.
     * @param os The output stream.
     * @param direction The Direction enum value to be printed.
     * @return The output stream with the Direction value appended.
     */
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

    /**
     * Calculates the recursion depth of a node in a binary tree based on its node ID.
     * @param nodeId The ID of the node in the binary tree.
     * @return The recursion depth of the node.
     */
    size_t recursionDepth(const size_t nodeId) {
        //t: depth of the current node, N: amount of nodes in the tree if tree is complete
        // $ N = 2^(t+1)-1 $ (geometric series formula for partial sums), assume $ N >= idx + 1 $
        return static_cast<size_t>(std::ceil(std::log2(nodeId + 2))) - 1;
    }
}// namespace kdtree
