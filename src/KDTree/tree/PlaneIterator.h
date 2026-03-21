#pragma once
#include "KDTree/tree/TreeNode.h"
#include "KDTree/tree/SplitNode.h"

#include <iterator>
#include <queue>

namespace kdtree {
    class PlaneIterator {
        private:
        std::queue<std::shared_ptr<SplitNode>> _nodeQueue{};
        public:
        using value_type = std::pair<Plane, Box>;
        using reference = value_type;
        using const_reference = value_type;
        using pointer = void;
        using iterator_category = std::input_iterator_tag;
        PlaneIterator() = default;
        explicit PlaneIterator(std::shared_ptr<SplitNode> rootNode);
        const_reference operator*() const;
        PlaneIterator& operator++();
        PlaneIterator operator++(int);
        bool operator==(const PlaneIterator &rhs) const;
        bool operator!=(const PlaneIterator &rhs) const;
    };
}
