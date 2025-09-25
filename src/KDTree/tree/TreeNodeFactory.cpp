#include "KDTree/tree/TreeNodeFactory.h"

    namespace kdtree::TreeNodeFactory {
        std::unique_ptr<TreeNode> createTreeNode(const SplitParam &splitParam, size_t nodeId) {
            //avoid splitting after certain tree depth
            if (recursionDepth(nodeId) >= MAX_RECURSION_DEPTH) {
                return std::make_unique<LeafNode>(splitParam, nodeId);
            }
            const size_t numberOfPoints{splitParam.boundPoints.size()};
            //find optimal plane splitting this node's bounding box
            auto [plane, planeCost, pointLists] = splitParam.planeSelectionStrategy->findPlane(splitParam);
            const double costWithoutSplit = static_cast<double>(numberOfPoints) * PlaneSelectionAlgorithm::pointCost;

            const auto pointsInMinimalBox = pointLists[0]->size();
            const auto pointsInMaximalBox = pointLists[1]->size();
            // Check if the boxes are divided into smaller regions
            const bool splitFailsToReduceSize = std::isinf(planeCost) || numberOfPoints <= pointsInMinimalBox + pointsInMaximalBox && (pointsInMinimalBox == 0 || pointsInMaximalBox == 0);
            //if the cost of splitting this node further is greater than just traversing the bound triangles or splitting does not reduce the amount of work in the resulting sub boxes, then don't split and return a LeafNode
            if (planeCost > costWithoutSplit || splitFailsToReduceSize) {
                return std::make_unique<LeafNode>(splitParam, nodeId);
            }
            //if not more costly, perform the split
            return std::make_unique<SplitNode>(splitParam, plane, pointLists, nodeId);
        }
    } // namespace kdtree::TreeNodeFactory

