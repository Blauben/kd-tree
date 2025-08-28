#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <variant>

#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/LeafNode.h"
#include "KDTree/tree/SplitNode.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/tree/TreeNode.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"

namespace kdtree {
struct SplitParam;
}  // namespace kdtree

constexpr uint8_t MAX_RECURSION_DEPTH{64};

    /**
     * Factory class for building TreeNodes. {@link TreeNode}
     */
    namespace kdtree::TreeNodeFactory {
        /**
        * Builds a new TreeNode for a KDTree. {@link KDTree}
        * @param splitParam Parameters for intersection testing and child node creation. {@link SplitParam}
        * @param nodeId The unique id to be assigned to the newly created node. Follows the convention that the left child gets the id 2 * <current_id> + 1 and
        * the right child 2 * <currrent_id> + 2.
        * @return A unique pointer to the new TreeNode.
         */
        std::unique_ptr<TreeNode> createTreeNode(const SplitParam &splitParam, size_t nodeId);
    } // namespace kdtree::TreeNodeFactory

