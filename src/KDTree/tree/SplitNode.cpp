#include "KDTree/tree/SplitNode.h"

namespace kdtree {
    SplitNode::SplitNode(const SplitParam &splitParam, const Plane &plane,
                         std::variant<ObjectIndexVectors<2>, PlaneEventVectors<2>> &shapeIndexLists,
                         const size_t nodeId)
        : TreeNode(splitParam, nodeId), _plane{plane}, _boundingBox{splitParam.boundingBox},
          _shapeLists{std::move(shapeIndexLists)} {

        LOG_DEBUG("SplitNode: Constructed with nodeId " + std::to_string(nodeId));
    }

    std::shared_ptr<TreeNode> SplitNode::getChildNode(const size_t index) {
        LOG_DEBUG("SplitNode: getChildNode(", std::to_string(index), ") called for nodeId ", std::to_string(this->nodeId));
        //create a reference to store the built node in
        std::shared_ptr<TreeNode> &node = index == 0 ? _lesser : _greater;
        //node is not yet built
        std::call_once(_childNodeCreated[index], [this, &node, &index] {
            LOG_DEBUG("SplitNode: Building child " + std::string(index == 0 ? "min" : "max") + " node for nodeId " + std::to_string(this->nodeId));
            //copy parent param and modify to fit new node
            SplitParam childParam{*_splitParam};
            //get the bounding box after splitting;
            auto [lesserBox, greaterBox] = this->_boundingBox.splitBox(this->_plane);
            childParam.boundingBox = index == 0 ? lesserBox : greaterBox;
            //get the shapes of the box
            std::visit([&childParam, index](auto &typeLists) -> void {
                childParam.boundObjects = *std::move(typeLists[index]);
            },
                       _shapeLists);
            childParam.splitDirection = static_cast<Direction>(
                    (static_cast<int>(childParam.splitDirection) + 1) % constants::DIMENSIONS);
            //increase the recursion depth of the direct child by 1
            node = TreeNodeFactory::createTreeNode(childParam, 2 * nodeId + 1 + index);
            //whichever call is the second to finish is guaranteed (via the atomic's acquire-release ordering) to see
            //the other call's writes, so it can safely free the now-unneeded splitParam exactly once.
            if (_builtChildCount.fetch_add(1, std::memory_order_acq_rel) == 1) {
                _splitParam.reset();
            }
        });
        return node;
    }

    std::vector<std::shared_ptr<TreeNode>> SplitNode::getChildrenForIntersection(
            const Vertex &origin, const Vertex &ray, const Vertex &inverseRay) {
        LOG_DEBUG("SplitNode: getChildrenForIntersection called for nodeId ", std::to_string(this->nodeId));
        using namespace kdtree::util;
        std::vector<std::shared_ptr<TreeNode>> delegates{};
        //a SplitNode has max two children, so no more space needed.
        delegates.reserve(2);
        //calculate entry and exit points of the ray hitting the bounding box
        auto [t_enter, t_exit] = _boundingBox.rayBoxIntersection(origin, inverseRay);
        // bounding box was not hit because the ray passed the box or is moving into the opposite direction of it. Tolerance is used to avoid numerical issues when the ray is very close to the box but not hitting it. Negliecting the tolerance would lead to a situation where the ray is very close to the box but not hitting it, which would result in an empty intersection set. This is not desired behavior, as it would lead to missing potential intersections with shapes contained in the box.
        if (t_exit < t_enter - constants::EPSILON_NUMERICAL_TOLERANCE || t_exit < 0) {
            LOG_DEBUG("SplitNode: Ray missed bounding box for nodeId ", std::to_string(this->nodeId));
            //empty
            return delegates;
        }
        //calculate point where plane was hit
        const double t_split{_plane.rayPlaneIntersection(origin, inverseRay)};
        //the split plane is hit inside of the bounding box -> both child boxes need to be checked
        const bool isParallel = std::isinf(t_split);
        bool planeIsHitInsideBox = 0 <= t_split && t_enter - constants::EPSILON_NUMERICAL_TOLERANCE <= t_split && t_split <= t_exit + constants::EPSILON_NUMERICAL_TOLERANCE;
        if (!isParallel && planeIsHitInsideBox) {
            LOG_DEBUG("SplitNode: Plane hit inside bounding box for nodeId ", std::to_string(this->nodeId));
            delegates.push_back(getChildNode(0));
            delegates.push_back(getChildNode(1));
            return delegates;
        }
        // getChildNode(0) is the lesser child, getChildNode(1) the greater one, so originIsBigger
        // maps directly to the child index without needing to invert it.
        const bool originIsBigger = origin[static_cast<int>(_plane.orientation)] >= _plane.axisCoordinate;
        // The ray never reaches the split plane - either it is parallel to it (t_split is +-inf, isParallel) or the plane lies behind the ray's origin (t_split < 0). Take the box closer to the origin
        if (isParallel || t_split < 0) {
            LOG_DEBUG("SplitNode: Ray parallel to or split plane behind ray origin for nodeId ", std::to_string(this->nodeId));
            delegates.push_back(getChildNode(static_cast<size_t>(originIsBigger)));
            return delegates;
        }
        // At this point only one box is hit and the split plane is crossed by the ray somewhere in this space: either the one entered before the split plane is
        // reached (t_split > t_exit) or the one entered after (t_split < t_enter). Normally boxOnOriginSide in combination with originIsBigger would be enough to determine which box is hit. However the origin could lie on the split plane which would make the determination ambiguous.
        // Check if the origin lies on the split plane and if so, return the box that is hit by the ray. The box that is hit by the ray is determined by the direction of the ray and the position of the box relative to the origin.
        if (std::abs(origin[static_cast<int>(_plane.orientation)] - _plane.axisCoordinate) < constants::EPSILON_NUMERICAL_TOLERANCE) {
            LOG_DEBUG("SplitNode: Ray origin lies on split plane for nodeId ", std::to_string(this->nodeId));
            const bool rayIncreasesOnAxis = ray[static_cast<int>(_plane.orientation)] > 0;
            // if the ray is increasing on the axis of the split plane, the box that is hit is the one that is greater than the split plane. Return box with index 1.
            delegates.push_back(getChildNode(static_cast<size_t>(rayIncreasesOnAxis)));
            return delegates;
        }
        const bool boxOnOriginSide = t_split > t_exit;
        const bool boxIsGreater = boxOnOriginSide ? originIsBigger : !originIsBigger;
        const size_t childIndex = static_cast<size_t>(boxIsGreater);
        LOG_DEBUG("SplitNode: Ray hits ", childIndex == 0 ? "lesser" : "greater", " box for nodeId ", std::to_string(this->nodeId));
        delegates.push_back(getChildNode(childIndex));
        return delegates;
    }

    std::string SplitNode::toString() const {
        std::stringstream sstream{};
        sstream << "SplitNode ID:  " << this->nodeId << ", Depth: " << recursionDepth(this->nodeId) << ", Plane: " << this->_plane << std::endl;
        sstream << "Children; Lesser: " << (this->_lesser != nullptr ? std::to_string(this->_lesser->nodeId) : "None")
                << "; Greater: " << (this->_greater != nullptr ? std::to_string(this->_greater->nodeId) : "None") << std::endl;
        if (this->_lesser != nullptr) {
            sstream << *this->_lesser;
        }
        if (this->_greater != nullptr) {
            sstream << *this->_greater;
        }
        return sstream.str();
    }

    std::ostream &operator<<(std::ostream &os, const SplitNode &node) {
        std::cout << node.toString();

        return os;
    }
}// namespace kdtree
