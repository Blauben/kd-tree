#include "KDTree/tree/NodeRegister.h"

#include "LeafNode.h"

namespace kdtree{

    void NodeRegister::registerLeafNode(const std::shared_ptr<LeafNode> &node) {
        std::unique_lock lock(leafNodeMutex);
        leafNodes.push_back(node);
    }

    void NodeRegister::unregisterLeafNode(const std::shared_ptr<LeafNode> &node) {
        std::unique_lock lock(leafNodeMutex);
        // Remove the weak_ptr pointing to this node by comparing with the shared_ptr
        // This avoids calling expired() on weak_ptrs, which can be unsafe during destruction
        std::erase_if(leafNodes, [&node](const std::weak_ptr<LeafNode> &weakPtr) {
            // safe_ptr will be nullptr if the weak_ptr is expired or doesn't match
            auto safe_ptr = weakPtr.lock();
            return safe_ptr.get() == node.get();
        });
    }

    void NodeRegister::removeExpired() {
        // Cleanup expired weak_ptr entries from the registry
        std::unique_lock lock(leafNodeMutex);
        std::erase_if(leafNodes, [](const std::weak_ptr<LeafNode> &ptr) {
            return ptr.expired();
        });
    }
}


