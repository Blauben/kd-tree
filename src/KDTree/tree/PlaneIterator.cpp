#include "PlaneIterator.h"

namespace kdtree {
    PlaneIterator::PlaneIterator(std::shared_ptr<SplitNode> rootNode) {
        if (rootNode != nullptr) {
            _nodeQueue.emplace(rootNode);
        }
    }

    PlaneIterator::const_reference PlaneIterator::operator*() const {
        const auto &node = _nodeQueue.front();
        return {node->_plane, node->_boundingBox};
    }

    PlaneIterator & PlaneIterator::operator++() {
        // if the queue is empty no advancements can be made, return immediately
        if (_nodeQueue.empty()) {
            return *this;
        }
        // queue child nodes of the current element
        const auto splitNode = _nodeQueue.front();
        for (size_t i = 0; i < 2; ++i) {
            if (auto childNode = std::dynamic_pointer_cast<SplitNode>(splitNode->getChildNode(i))) {
                _nodeQueue.push(childNode);
            }
        }
        // drop the current element
        _nodeQueue.pop();
        return *this;
    }

    PlaneIterator PlaneIterator::operator++(int) {
        auto currentState =  PlaneIterator(*this);
        this->operator++();
        return currentState;
    }

    bool PlaneIterator::operator==(const PlaneIterator &rhs) const {
        if (_nodeQueue.empty() && rhs._nodeQueue.empty()) {
            return true;
        }
        if (_nodeQueue.empty() || rhs._nodeQueue.empty()) {
            return false;
        }
        if (_nodeQueue.front() != rhs._nodeQueue.front()) {
            return false;
        }
        return true;
    }

    bool PlaneIterator::operator!=(const PlaneIterator &rhs) const {
        return !(*this == rhs);
    }
}
