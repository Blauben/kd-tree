#include "KDTree/tree/LeafNode.h"

namespace kdtree {
    LeafNode::LeafNode(const SplitParam &splitParam, const size_t nodeId)
        : TreeNode(splitParam, nodeId) {
        LOG_DEBUG("LeafNode: Constructed with nodeId " + std::to_string(nodeId));
    }

    LeafNode::~LeafNode() {
        LOG_DEBUG("LeafNode: Destroyed with nodeId " + std::to_string(nodeId));
        // Do NOT call removeExpired() here - it attempts to call expired() on weak_ptrs
        // while the control block of the dying node may be invalid, causing segfaults.
        // Periodic cleanup should be done elsewhere (e.g., in tree destruction or separate cleanup calls).
    }

    bool LeafNode::needTreeRebuild() {
        convertPlaneEventsToGeometry();
        for (const auto &handle: std::get<ObjectIndexVector>(_splitParam->boundObjects)) {
            const GeometryObject &object = _splitParam->geometryObjects[handle];
            for (const auto &vertex: object.getVertices()) {
                // TODO: set tolerance in relation to whole tree
                if (!_splitParam->boundingBox.isVertexInBox(vertex, constants::EPSILON_VERTEX_BOX_TOLERANCE)) {
                    return true;
                }
            }
        }
        return false;
    }

    void LeafNode::convertPlaneEventsToGeometry() {
        if (std::holds_alternative<PlaneEventVector>(_splitParam->boundObjects)) {
            std::call_once(_convertedToObjects, [this] {
                LOG_DEBUG("LeafNode: Converting PlaneEventVector to Geometry for nodeId " + std::to_string(this->nodeId));
                _splitParam->boundObjects = convertEventsToGeometry(std::get<PlaneEventVector>(_splitParam->boundObjects));
            });
        }
    }

    void LeafNode::getIntersections(const Vertex &origin, const Vertex &ray,
                                    std::set<Vertex> &intersections) {
        LOG_DEBUG("LeafNode: getIntersections called for nodeId ", std::to_string(this->nodeId));
        convertPlaneEventsToGeometry();
        const ObjectIndexVector &boundObjects{std::get<ObjectIndexVector>(_splitParam->boundObjects)};
        //traverses all contained faces and performs intersection tests with them -> store results in the buffer passed in the arguments
        auto transform_op = [this, &ray, &origin, &intersections](const size_t objIndex) -> std::optional<Vertex> {
            const std::optional<Vertex> intersection = rayIntersectsObject(
                    origin, ray, _splitParam->geometryObjects[objIndex]);
            LOG_DEBUG("LeafNode: getIntersections finished for nodeId " + std::to_string(this->nodeId));
            return intersection;
        };

        // Create transform iterators for the bound objects and perform intersection tests in them
        auto tbegin = thrust::make_transform_iterator(boundObjects.begin(), transform_op);
        auto tend = thrust::make_transform_iterator(boundObjects.end(), transform_op);

        thrust::host_vector<std::optional<Vertex>> out(boundObjects.size());// upper bound

        // execute thrust::copy_if on device or host based on the number of objects in the leaf node
        auto out_end = boundObjects.size() >= constants::LEAF_THRUST_PARALLEL_THRESHOLD
                               ? thrust::copy_if(thrust::device, tbegin, tend, out.begin(),
                                                 [](const std::optional<Vertex> &i) { return i.has_value(); })
                               : thrust::copy_if(thrust::host, tbegin, tend, out.begin(),
                                                 [](const std::optional<Vertex> &i) { return i.has_value(); });

        out.resize(out_end - out.begin());
        // Insert the valid intersections into the output set
        std::transform(out.begin(), out.end(), std::inserter(intersections, intersections.end()), [](const std::optional<Vertex> &opt) { return opt.value(); });
    }

    std::optional<Vertex> LeafNode::rayIntersectsObject(const Vertex &rayOrigin, const Vertex &rayVector,
                                                        const GeometryObject &object) {
        if (object.getIndexVector().size() == 3) {
            return rayIntersectsTriangle(rayOrigin, rayVector, object.getVertices());
        }
        if (object.getIndexVector().size() == 1) {
            return rayIntersectsPoint(rayOrigin, rayVector, object.getVertices()[0]);
        }
        LOG_ERROR("LeafNode: rayIntersectsObject called with invalid object (neither triangle nor point)");
        throw std::runtime_error("Error: rayIntersectsObject called with neither a triangle nor a point.");
    }

    std::optional<Vertex> LeafNode::rayIntersectsTriangle(const Vertex &rayOrigin, const Vertex &rayVector,
                                                          const std::vector<Vertex> &triangleVertices) {
        // Adapted Möller–Trumbore intersection algorithm
        // see https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
        using namespace kdtree;
        using namespace util;
        const Vertex edge1 = triangleVertices[1] - triangleVertices[0];
        const Vertex edge2 = triangleVertices[2] - triangleVertices[0];
        const Vertex h = cross(rayVector, edge2);
        const double a = dot(edge1, h);
        if (a > -constants::EPSILON_ZERO_OFFSET && a < constants::EPSILON_ZERO_OFFSET) {
            return std::nullopt;
        }

        const double f = 1.0 / a;
        const Vertex s = rayOrigin - triangleVertices[0];
        const double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0) {
            return std::nullopt;
        }
        const Vertex q = cross(s, edge1);
        const double v = f * dot(rayVector, q);
        if (v < 0.0 || u + v > 1.0) {
            return std::nullopt;
        }

        const double t = f * dot(edge2, q);
        if (t > constants::EPSILON_ZERO_OFFSET) {
            return rayOrigin + rayVector * t;
        }
        return std::nullopt;
    }

    std::optional<Vertex> LeafNode::rayIntersectsPoint(const Vertex &rayOrigin, const Vertex &rayVector, const Vertex &center) {
        using namespace util;

        // Vector from ray origin to sphere center
        const Vertex oc = rayOrigin - center;

        // Calculate the nearest intersection point
        const double t = -dot(rayVector, oc) / dot(rayVector, rayVector);

        const Vertex intersection = rayOrigin + rayVector * t;

        const double distance = dot(intersection - center, intersection - center);
        if (distance > constants::EPSILON_RAY_POINT_OFFSET) {
            return std::nullopt;
        }

        // Return the intersection point
        return intersection;
    }

    std::string LeafNode::toString() const {
        std::stringstream sstream{};
        sstream << "LeafNode ID: " << this->nodeId << ", Depth: " << recursionDepth(this->nodeId) << std::endl;
        return sstream.str();
    }

    std::ostream &operator<<(std::ostream &os, const LeafNode &node) {
        os << node.toString();
        return os;
    }
}// namespace kdtree
