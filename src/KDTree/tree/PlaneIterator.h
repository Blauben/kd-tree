#pragma once
#include "KDTree/tree/SplitNode.h"
#include "KDTree/tree/TreeNode.h"

#include <iterator>
#include <queue>

namespace kdtree {
    /**
     * This class implements an input iterator for the planes of a KDTree. It performs a breadth-first traversal of the tree and returns the plane and bounding box of each SplitNode it encounters. The iterator is initialized with the root node of the tree and uses a queue to keep track of the nodes to be processed. When the iterator is incremented, it processes the current node, queues its child nodes if they are SplitNodes, and then moves to the next node in the queue. The iteration ends when there are no more nodes to process (i.e., when the queue is empty).
     */
    class PlaneIterator {
    private:
        std::queue<std::shared_ptr<SplitNode>> _nodeQueue{};

    public:
        // standard iterator definitions
        using value_type = std::pair<Plane, Box>;
        using reference = value_type;
        using const_reference = value_type;
        using pointer = void;
        using iterator_category = std::input_iterator_tag;
        PlaneIterator() = default;
        explicit PlaneIterator(std::shared_ptr<SplitNode> rootNode);
        const_reference operator*() const;
        PlaneIterator &operator++();
        PlaneIterator operator++(int);
        bool operator==(const PlaneIterator &rhs) const;
        bool operator!=(const PlaneIterator &rhs) const;
    };
}// namespace kdtree
