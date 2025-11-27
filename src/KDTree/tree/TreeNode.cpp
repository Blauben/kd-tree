#include "KDTree/tree/TreeNode.h"

namespace kdtree {
    TreeNode::TreeNode(const SplitParam &splitParam, const size_t nodeId)
        : nodeId{nodeId}, _splitParam{std::make_unique<SplitParam>(splitParam)} {
#ifdef DEBUG
        constexpr unsigned long geometryIndex = 0;
        const auto& bound = std::get<ObjectIndexVector>(this->_splitParam->boundObjects);
        if (std::find(bound.cbegin(), bound.cend(), geometryIndex) != bound.end()) {
            DEBUG("Traced geoObject " << geometryIndex << " to Node: " << this->nodeId);
        }
#endif
    }

    std::ostream &operator<<(std::ostream &os, const TreeNode &node) {
        os << node.toString();
        return os;
    }

}// namespace kdtree