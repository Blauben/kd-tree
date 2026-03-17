
#include "KDTree/tree/KDTree.h"
#include "KDTree/Logging.h"

namespace kdtree {
    //on initialization of the tree a single bounding box which includes all the shapes of the polyhedron is generated. Both the list of included shapes and the parameters of the box are written to the split parameters
    KDTree::KDTree(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &shapes,
                   const PlaneSelectionAlgorithm::Algorithm algorithm) {
        LOG_INFO("KDTree: Constructing from vertices and faces");
        GeometryObject::vertices = vertices;
        _geometryObjects.reserve(shapes.size());
        //transform shape indices to GeometryObjects
        std::ranges::for_each(shapes, [this](const IndexVector &vertexIndices) {
            _geometryObjects.emplace_back(vertexIndices);
        });
        _splitParam = std::make_unique<SplitParam>(_geometryObjects, Box::getBoundingBox(vertices), Direction::X,
                                                   PlaneSelectionAlgorithmFactory::create(algorithm));
        LOG_DEBUG("KDTree: Construction complete, split parameters initialized");
    }

    KDTree::KDTree(const std::vector<Vertex> &particles, const PlaneSelectionAlgorithm::Algorithm algorithm) : KDTree{
                                                                                                                       particles,
                                                                                                                       [&particles] {
                                                                                                                           //each particle is represented by a single vertex, so each shape only contains one vertex index
                                                                                                                           std::vector<IndexVector> shapes{};
                                                                                                                           shapes.reserve(particles.size());
                                                                                                                           for (size_t index = 0; index < particles.size(); index++) {
                                                                                                                               shapes.emplace_back(1, index);
                                                                                                                           }
                                                                                                                           return shapes;
                                                                                                                       }(),
                                                                                                                       algorithm} {
        LOG_INFO("KDTree: Constructed from particles");
    }

    KDTree::KDTree(const std::tuple<std::vector<Vertex>, std::vector<IndexVector>> &polySource,
                   const PlaneSelectionAlgorithm::Algorithm algorithm)
        : KDTree(std::get<0>(polySource), std::get<1>(polySource), algorithm) {
    }


    KDTree::KDTree(const std::string &nodeFilePath, const std::string &faceFilePath, const PlaneSelectionAlgorithm::Algorithm algorithm) : KDTree(TetgenAdapter{{nodeFilePath, faceFilePath}}.getPolyhedralSource(), algorithm) {
        LOG_INFO("KDTree: Data fetched from node and face file paths");
    }

    std::shared_ptr<TreeNode> KDTree::getRootNode() {
        LOG_DEBUG("KDTree: getRootNode called");
        //if the node has already been generated, don't do it again. Let the factory determine the TreeNode subclass based on the optimal split.
        std::call_once(_rootNodeCreated, [this] {
            LOG_DEBUG("KDTree: Creating root node via TreeNodeFactory");
            this->_rootNode = TreeNodeFactory::createTreeNode(*std::move(_splitParam), 0);
        });
        return this->_rootNode;
    }

    size_t KDTree::countIntersections(const Vertex &origin, const Vertex &ray) {
        //it's possible that a single intersection point is on the edge between two shapes. The point would be counted twice if the intersection points were not documented -> use of std::set
        LOG_DEBUG("KDTree: Counting intersections");
        //it's possible that a single intersection point is on the edge between two triangles. The point would be counted twice if the intersection points were not documented -> use of std::set
        std::set<Vertex> set{};
        this->getIntersections(origin, ray, set);
        LOG_INFO("KDTree: Intersections counted: " + std::to_string(set.size()));
        return set.size();
    }

    void KDTree::getIntersections(const Vertex &origin, const Vertex &ray, std::set<Vertex> &intersections) {
        LOG_DEBUG("KDTree: getIntersections called");
        //iterative approach to avoid stack and heap overflows
        //queue for children of processed nodes
        std::deque<std::shared_ptr<TreeNode>> queue{};
        //calculate inverse ray direction
        const Vertex inverseRay{1. / ray[0], 1. / ray[1], 1. / ray[2]};
        //init with tree root
        queue.push_back(getRootNode());
        while (!queue.empty()) {
            auto node = queue.front();
            //if node is SplitNode perform intersection checks on the children and queue them accordingly
            if (const auto split = std::dynamic_pointer_cast<SplitNode>(node)) {
                LOG_DEBUG("KDTree: SplitNode with nodeId ", std::to_string(split->nodeId), " encountered in getIntersections");
                const auto children = split->getChildrenForIntersection(origin, ray, inverseRay);
                std::ranges::for_each(children, [&queue](const auto &child) {
                    queue.push_back(child);
                });
            }
            //if node is leaf then perform intersections with the shapes contained
            else if (const auto leaf = std::dynamic_pointer_cast<LeafNode>(node)) {
                LOG_DEBUG("KDTree: LeafNode with nodeId ", std::to_string(leaf->nodeId), " encountered in getIntersections");
                leaf->getIntersections(origin, ray, intersections);
            }
            queue.pop_front();
        }
        LOG_INFO("KDTree: getIntersections finished");
    }

    KDTree &KDTree::prebuildTree() {
        LOG_INFO("KDTree: prebuildTree called");
        //queue for children of processed nodes
        std::deque<std::shared_ptr<TreeNode>> queue{};
        //subsequently call getter functions for the root node and all child nodes to initiate a full build of the tree
        queue.push_back(getRootNode());
        while (!queue.empty()) {
            auto node = queue.front();
            //if node is SplitNode perform intersection checks on the children and queue them accordingly
            if (const auto split = std::dynamic_pointer_cast<SplitNode>(node)) {
                LOG_DEBUG("KDTree: SplitNode encountered in prebuildTree");
                //build child nodes and add them to the queue
                queue.push_back(split->getChildNode(0));
                queue.push_back(split->getChildNode(1));
            }
            //remove the processed node as its direct children have been built by getChildNode
            queue.pop_front();
        }
        LOG_INFO("KDTree: prebuildTree finished");
        return *this;
    }

    std::ostream &operator<<(std::ostream &os, const KDTree &kdTree) {
        os << to_string(kdTree);
        return os;
    }

    std::string to_string(const KDTree &kdTree) {
        std::ostringstream os{};
        if (kdTree._rootNode != nullptr) {
            os << *kdTree._rootNode;
        } else {
            os << "KDTree rootNode is empty!";
        }
        return os.str();
    }
}// namespace kdtree
