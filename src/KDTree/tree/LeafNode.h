#pragma once

#include "KDTree/Logging.h"
#include "KDTree/tree/KdDefinitions.h"
#include "KDTree/tree/SplitParam.h"
#include "KDTree/tree/TreeNode.h"
#include "KDTree/util/UtilityContainer.h"
#include "KDTree/util/UtilityFloatArithmetic.h"
#include "thrust/detail/execution_policy.h"
#include "thrust/execution_policy.h"
#include "thrust/system/detail/sequential/for_each.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <optional>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

namespace kdtree {
    struct SplitParam;

    /**
     * A TreeNode contained in a KDTree that doesn't split the spatial hierarchy any further. Intersection tests are directly performed on the contained shapes here.
     */
    class LeafNode final : public TreeNode {
    public:
        /**
         * Takes parameters from the parent node and stores them for later intersection tests.
         * @param splitParam Parameters produced during the split that resulted in the creation of this node.
         * @param nodeId Unique Id given by the TreeNodeFactory.
         */
        explicit LeafNode(const SplitParam &splitParam, size_t nodeId);
        LeafNode(const LeafNode&) = delete;
        LeafNode& operator=(const LeafNode&) = delete;
        LeafNode(LeafNode&&) noexcept = delete;
        LeafNode& operator=(LeafNode&&) noexcept = delete;
        ~LeafNode() override;
        static std::vector<std::weak_ptr<LeafNode>> leafNodes;

        /**
         * Register a weak_ptr to this LeafNode in the global leafNodes registry.
         * Should be called by TreeNodeFactory after creating a shared_ptr.
         */
        static void registerLeafNode(const std::shared_ptr<LeafNode>& node);

        /**
        * Used to calculated intersections of a ray and the polyhedron's shapes contained in this node.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @param intersections The set intersection points are added to.
        */
        void getIntersections(const Vertex &origin, const Vertex &ray, std::set<Vertex> &intersections);

        [[nodiscard]] std::string toString() const override;

        friend std::ostream &operator<<(std::ostream &os, const LeafNode &node);

    private:
        /**
         * Möller-Trumbore Algorithm for Ray-Face intersection.
         * @param rayOrigin The point where the ray originates from.
         * @param rayVector Specifies the ray direction.
         * @param object the face to test against, described by the vertices that comprise it (passed by index reference).
         * @return
         */
        static std::optional<Vertex> rayIntersectsObject(const Vertex &rayOrigin, const Vertex &rayVector,
                                                         const GeometryObject &object);

        /**
         * Möller-Trumbore Algorithm for Ray-Face intersection.
         * @param rayOrigin The point where the ray originates from.
         * @param rayVector Specifies the ray direction.
         * @param triangleVertices the face to test against, described by the vertices that comprise it (passed by value).
         * @return
         */
        static std::optional<Vertex> rayIntersectsTriangle(const Vertex &rayOrigin, const Vertex &rayVector,
                                                           const std::vector<Vertex> &triangleVertices);

        /**
         * Ray-Point intersection test. Also counts the intersection if the ray passes the point with distance smaller than a specified epsilon.
         * @param rayOrigin The point where the ray originates from.
         * @param rayVector Specifies the ray direction.
         * @param center the point to test against.
         * @return The intersection point if an intersection was found.
         */
        static std::optional<Vertex> rayIntersectsPoint(const Vertex &rayOrigin, const Vertex &rayVector, const Vertex &center);

        /**
         * Flags set when _splitParam boundObjects are converted from PlaneEvents to shapes
         */
        std::once_flag _convertedToObjects;
    };
}// namespace kdtree
