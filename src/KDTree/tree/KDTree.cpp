#include "KDTree/tree/KDTree.h"

#include "KDTree/input/TetgenAdapter.h"

namespace kdtree {
    //on initialization of the tree a single bounding box which includes all the faces of the polyhedron is generated. Both the list of included faces and the parameters of the box are written to the split parameters
    KDTree::KDTree(const PointVector &points,
                   const PlaneSelectionAlgorithm::Algorithm algorithm)
        : _points{points},
          _splitParam{
              std::make_unique<SplitParam>(_points, Box::getBoundingBox(_points), Direction::X,
                                           PlaneSelectionAlgorithmFactory::create(algorithm))
          } {
        _splitParam->boundPoints.resize(points.size());
        std::iota(_splitParam->boundPoints.begin(), _splitParam->boundPoints.end(), 0);
    }

    std::shared_ptr<TreeNode> KDTree::getRootNode() {
        //if the node has already been generated, don't do it again. Let the factory determine the TreeNode subclass based on the optimal split.
        std::call_once(_rootNodeCreated, [this] {
            this->_rootNode = TreeNodeFactory::createTreeNode(*std::move(_splitParam), 0);
        });
        return this->_rootNode;
    }

    KDTree &KDTree::prebuildTree() {
        //queue for children of processed nodes
        std::deque<std::shared_ptr<TreeNode> > queue{};
        //subsequently call getter functions for the root node and all child nodes to initiate a full build of the tree
        queue.push_back(getRootNode());
        while (!queue.empty()) {
            auto node = queue.front();
            //if node is SplitNode perform intersection checks on the children and queue them accordingly
            if (const auto split = std::dynamic_pointer_cast<SplitNode>(node)) {
                //build child nodes and add them to the queue
                queue.push_back(split->getChildNode(0));
                queue.push_back(split->getChildNode(1));
            }
            //remove the processed node as its direct children have been built by getChildNode
            queue.pop_front();
        }
        return *this;
    }

    std::ostream &operator<<(std::ostream &os, const KDTree &kdTree) {
        if (kdTree._rootNode != nullptr) {
            os << *(kdTree._rootNode);
        } else {
            os << "KDTree rootNode is empty!";
        }
        return os;
    }
} // namespace kdtree
