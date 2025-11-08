#include "KDTree/tree/LeafNode.h"

namespace kdtree {
    LeafNode::LeafNode(const SplitParam &splitParam, const size_t nodeId)
        : TreeNode(splitParam, nodeId) {
    }

    void LeafNode::getIntersections(const Array3 &origin, const Array3 &ray,
                                        std::set<Array3> &intersections) {
        if (std::holds_alternative<PlaneEventVector>(_splitParam->boundObjects)) {
            std::call_once(convertedToFace, [this]() {
                _splitParam->boundObjects = convertEventsToGeometry(std::get<PlaneEventVector>(_splitParam->boundObjects));
            });
        }
        std::mutex writeLock{};
        const ObjectIndexVector &boundObjects{std::get<ObjectIndexVector>(_splitParam->boundObjects)};
        std::vector<Array3> results(boundObjects.size());
        //traverses all contained faces and performs intersection tests with them -> store results in the buffer passed in the arguments
        thrust::for_each(thrust::device, boundObjects.cbegin(), boundObjects.cend(),
                         [this, &ray, &origin, &intersections, &writeLock](const size_t faceIndex) {
                             const std::optional<Array3> intersection = rayIntersectsObject(
                                     origin, ray, _splitParam->geometryObjects[faceIndex]);
                             if (intersection.has_value()) {
                                 std::unique_lock lock(writeLock);
                                 intersections.insert(intersection.value());
                             }
                         });
    }

    std::optional<Array3> LeafNode::rayIntersectsObject(const Array3 &rayOrigin, const Array3 &rayVector,
                                                          const GeometryObject &object) {
        if (object.getIndexVector().size() == 3) {
            return rayIntersectsTriangle(rayOrigin, rayVector, object.getVertices());
        }
        if (object.getIndexVector().size() == 1) {
            return rayIntersectsPoint(rayOrigin, rayVector, object.getVertices()[0]);
        }

        throw std::runtime_error("Error: rayIntersectsObject called with neither a triangle nor a point.");

    }

    std::optional<Array3> LeafNode::rayIntersectsTriangle(const Array3 &rayOrigin, const Array3 &rayVector,
                                                          const std::vector<Array3> &triangleVertices) {
        // Adapted Möller–Trumbore intersection algorithm
        // see https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
        using namespace kdtree;
        using namespace util;
        const Array3 edge1 = triangleVertices[1] - triangleVertices[0];
        const Array3 edge2 = triangleVertices[2] - triangleVertices[0];
        const Array3 h = cross(rayVector, edge2);
        const double a = dot(edge1, h);
        if (a > -EPSILON_ZERO_OFFSET && a < EPSILON_ZERO_OFFSET) {
            return std::nullopt;
        }

        const double f = 1.0 / a;
        const Array3 s = rayOrigin - triangleVertices[0];
        const double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0) {
            return std::nullopt;
        }

        const Array3 q = cross(s, edge1);
        const double v = f * dot(rayVector, q);
        if (v < 0.0 || u + v > 1.0) {
            return std::nullopt;
        }

        const double t = f * dot(edge2, q);
        if (t > EPSILON_ZERO_OFFSET) {
            return rayOrigin + rayVector * t;
        }
        return std::nullopt;
    }
    std::optional<Array3> LeafNode::rayIntersectsPoint(const Array3 &rayOrigin, const Array3 &rayVector, const Array3 &center) {
        using namespace util;
        const double EPSILON_OFFSET = 1e-9 /* TODO: get radius from object, or use a default small value */;

        // Vector from ray origin to sphere center
        const Array3 oc = rayOrigin - center;

        // Calculate the nearest intersection point
        const double t = - dot(rayVector, oc) / dot(rayVector, rayVector);

        const Array3 intersection = rayOrigin + rayVector * t;

        const double distance = std::sqrt(dot(intersection - center, intersection - center));
        if (distance > EPSILON_OFFSET) {
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
