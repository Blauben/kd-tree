#include "KDTree/tree/LeafNode.h"

namespace kdtree {
    LeafNode::LeafNode(const SplitParam &splitParam, const size_t nodeId)
        : TreeNode(splitParam, nodeId) {
    }

    std::string LeafNode::toString() const {
        std::stringstream sstream{};
        sstream << "LeafNode ID: " << this->nodeId << ", Depth: " << recursionDepth(this->nodeId) << std::endl;
        return sstream.str();
    }

    std::ostream &operator<<(std::ostream &os, const LeafNode &node) {
        os << node.toString();
        return os;
    }
}
