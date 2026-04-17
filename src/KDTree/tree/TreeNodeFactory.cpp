#include "KDTree/tree/TreeNodeFactory.h"

namespace kdtree::TreeNodeFactory {
    std::shared_ptr<TreeNode> createTreeNode(const SplitParam &splitParam, size_t nodeId) {
        LOG_DEBUG("TreeNodeFactory: createTreeNode called for nodeId ", std::to_string(nodeId));
        //avoid splitting after certain tree depth
        if (recursionDepth(nodeId) >= constants::MAX_RECURSION_DEPTH) {
            LOG_WARN("TreeNodeFactory: Max recursion depth reached, creating LeafNode for nodeId " + std::to_string(nodeId));
            auto leafNode = std::make_shared<LeafNode>(splitParam, nodeId);
            LeafNode::registerLeafNode(leafNode);
            return leafNode;
        }
        const size_t numberOfObjects{countGeometryObjects(splitParam.boundObjects)};
        //find optimal plane splitting this node's bounding box
        auto [plane, planeCost, shapeLists] = splitParam.planeSelectionStrategy->findPlane(splitParam);
        const double costWithoutSplit = static_cast<double>(numberOfObjects) * constants::SHAPE_INTERSECTION_COST;

        // Check if the boxes are divided into smaller regions
        const bool splitFailsToReduceSize = std::isinf(planeCost) || std::visit([numberOfObjects](auto &typeLists) {
                                                // Count shapes in each split box
                                                const size_t geometryInMinimalBox = countGeometryObjects(*typeLists[0]);
                                                const size_t geometryInMaximalBox = countGeometryObjects(*typeLists[1]);

                                                // Ensure that the split meaningfully divides shapes
                                                return numberOfObjects <= geometryInMinimalBox + geometryInMaximalBox && (geometryInMinimalBox == 0 || geometryInMaximalBox == 0);
                                            },
                                                                                shapeLists);
        //if the cost of splitting this node further is greater than just traversing the bound shapes or splitting does not reduce the amount of work in the resulting sub boxes, then don't split and return a LeafNode
        if (planeCost > costWithoutSplit || splitFailsToReduceSize) {
            LOG_DEBUG("TreeNodeFactory: Creating LeafNode for nodeId " + std::to_string(nodeId) + ", planeCost: " + std::to_string(planeCost) + ", costWithoutSplit: " + std::to_string(costWithoutSplit) + ", splitFailsToReduceSize: " + (splitFailsToReduceSize ? "true" : "false"));
            auto leafNode = std::make_shared<LeafNode>(splitParam, nodeId);
            LeafNode::registerLeafNode(leafNode);
            return leafNode;
        }
        //if not more costly, perform the split
        LOG_DEBUG("TreeNodeFactory: Creating SplitNode for nodeId " + std::to_string(nodeId));
        return std::make_shared<SplitNode>(splitParam, plane, shapeLists, nodeId);
    }
}// namespace kdtree::TreeNodeFactory
