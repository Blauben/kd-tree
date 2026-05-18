#include "KDTree/tree/TreeNode.h"

#include "KDTree/Info.h"
#include "KDTree/util/Constants.h"

namespace kdtree {
    TreeNode::TreeNode(const SplitParam &splitParam, const size_t nodeId)
        : nodeId{nodeId}, _splitParam{std::make_unique<SplitParam>(splitParam)} {
        if (KD_TREE_LOGGING_LEVEL == "DEBUG") {
            //debugging output to trace specific geometry objects through the tree
            const auto &bound = std::get<ObjectIndexVector>(this->_splitParam->boundObjects);
            if (std::find(bound.cbegin(), bound.cend(), constants::GEOMETRY_INDEX) != bound.end()) {
                LOG_DEBUG("Traced geoObject ", constants::GEOMETRY_INDEX, " to Node: ", this->nodeId);
            }
        }
    }

    TreeNode::TreeNode(TreeNode &&other) noexcept {
        nodeId = other.nodeId;
        this->_splitParam = std::move(other._splitParam);
        other.nodeId = SIZE_MAX;// sentinel: "I am moved-from"
    }

    TreeNode &TreeNode::operator=(TreeNode &&other) noexcept {
        if (this != &other) {
            nodeId = other.nodeId;
            _splitParam = std::move(other._splitParam);
            other.nodeId = SIZE_MAX;// sentinel: "I am moved-from"
        }
        return *this;
    }

    std::ostream &operator<<(std::ostream &os, const TreeNode &node) {
        os << node.toString();
        return os;
    }

}// namespace kdtree