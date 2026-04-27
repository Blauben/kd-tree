#include "KDTree/tree/NodeRegister.h"

#include "LeafNode.h"

namespace kdtree{

    void NodeRegister::registerLeafNode(const std::shared_ptr<LeafNode> &node) {
        std::unique_lock lock(leafNodeMutex);
        leafNodes.push_back(node);
    }

    void NodeRegister::removeExpired() {
        // Cleanup expired weak_ptr entries from the registry
        std::unique_lock lock(leafNodeMutex);
        std::erase_if(leafNodes, [](const std::weak_ptr<LeafNode> &ptr) {
            return ptr.expired();
        });
    }
}
