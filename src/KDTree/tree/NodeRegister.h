#pragma once
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

namespace kdtree {

    // forward declaration
    class LeafNode;

    class NodeRegister {
    public:
        NodeRegister() = default;
        ~NodeRegister() = default;

        /**
         * Stores pointers to leaf nodes for global access, e.g. to check if a tree rebuild is needed due to vertex movement outside the leaf node's bounding box. The pointers are stored as weak_ptrs to avoid memory leaks and dangling pointers when leaf nodes are destroyed.
         */
        std::vector<std::weak_ptr<LeafNode>> leafNodes{};

        std::shared_mutex leafNodeMutex{};

        /**
         * Register a weak_ptr to this LeafNode in the global leafNodes registry.
         * Should be called by TreeNodeFactory after creating a shared_ptr.
         */
        void registerLeafNode(const std::shared_ptr<LeafNode> &node);

        /**
         * Removes all expired weak_ptr references to already deleted LeafNodes from the leafNode register.
         */
        void removeExpired();

        /**
         * Explicitly unregisters a specific LeafNode from the leafNodes registry.
         * Should be called during LeafNode destruction to safely remove itself without
         * accessing invalid weak_ptr control blocks.
         */
        void unregisterLeafNode(const std::shared_ptr<LeafNode> &node);
    };
}// namespace kdtree
