#include "KDTree/tree/KDTree.h"

namespace kdtree {

    namespace {

#if defined(KD_TREE_OMP) or defined(KD_TREE_CPP)
        void buildChildNodes(const std::shared_ptr<TreeNode> &treeNode) {
            if (auto splitNode = std::dynamic_pointer_cast<SplitNode>(treeNode)) {
#pragma omp task
                {
                    buildChildNodes(splitNode->getChildNode(0));
                }
#pragma omp task
                {
                    buildChildNodes(splitNode->getChildNode(1));
                }
            }
        }
#endif

#if defined(KD_TREE_TBB)
        void buildChildNodes(const std::shared_ptr<TreeNode> &treeNode, tbb::task_group &tasks) {
            if (auto splitNode = std::dynamic_pointer_cast<SplitNode>(treeNode)) {
                tasks.run([splitNode, &tasks]() {
                    buildChildNodes(splitNode->getChildNode(0), tasks);
                });
                tasks.run([splitNode, &tasks]() {
                    buildChildNodes(splitNode->getChildNode(1), tasks);
                });
            }
        }
#endif
    }// namespace

    //on initialization of the tree a single bounding box which includes all the geometry objects is generated. Both the list of included objects and the parameters of the box are written to the split parameters
    KDTree::KDTree(const std::vector<Vertex> &vertices, const std::vector<IndexVector> &shapes,
                   const PlaneSelectionAlgorithm::Algorithm algorithm, const bool copyVertices, const bool particleMode)
        : _algorithm{algorithm}, _particleMode{particleMode} {
        LOG_DEBUG("KDTree: Start initialization");
        if (copyVertices) {
            _vertices = std::make_shared<std::vector<VertexHandle>>(vertices.begin(), vertices.end());
        } else {
            _vertices = std::make_shared<std::vector<VertexHandle>>();
            _vertices->reserve(vertices.size());
            std::for_each(vertices.begin(), vertices.end(), [this, copyVertices](const Vertex &vertex) {
                _vertices->emplace_back(&vertex);
            });
        }
        _geometryObjects.reserve(shapes.size());
        //transform shape indices to GeometryObjects
        std::ranges::for_each(shapes.cbegin(), shapes.cend(), [this, objectIndex = size_t{0}](const IndexVector &vertexIndices) mutable {
            _geometryObjects.emplace_back(vertexIndices, objectIndex++, _vertices);
        });
        _splitParam = std::make_unique<SplitParam>(_geometryObjects, Box::getBoundingBox(*_vertices), Direction::X,
                                                   PlaneSelectionAlgorithmFactory::create(algorithm), nodeRegister, particleMode);
        LOG_DEBUG("KDTree: Construction complete, split parameters initialized");
        // flush the logger so that the messages are picked up by test cases that check for log output during tree construction
        KDTreeLogger::defaultLogger().getLogger()->flush();
    }

    KDTree::KDTree(const std::vector<Vertex> &particles, const PlaneSelectionAlgorithm::Algorithm algorithm, const bool copyVertices)
        : KDTree{
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
                  algorithm, copyVertices, true} {
        LOG_DEBUG("KDTree: Constructed from particles");
    }

    KDTree::KDTree(std::tuple<std::vector<Vertex>, std::vector<IndexVector>> polySource,
                   const PlaneSelectionAlgorithm::Algorithm algorithm)
        : KDTree(std::get<0>(polySource), std::get<1>(polySource), algorithm) {
    }


    KDTree::KDTree(const std::string &nodeFilePath, const std::string &faceFilePath, const PlaneSelectionAlgorithm::Algorithm algorithm)
        : KDTree(TetgenAdapter{{nodeFilePath, faceFilePath}}.getPolyhedralSource(), algorithm) {
        LOG_DEBUG("KDTree: Data fetched from node and face file paths");
    }

    void KDTree::rebuildTree() {
        std::lock_guard lock(this->_rootNodeCreationMutex);
        this->_rootNodeCreated.store(false, std::memory_order_relaxed);//re-arm the lazy-build gate for the next getRootNode() call
        this->_rootNode.reset();                                       //reset the root node to allow rebuilding the tree
        nodeRegister.leafNodes.clear();
        // since splitParam are moved once the previous tree is built, they have to regenerated here
        _splitParam = std::make_unique<SplitParam>(_geometryObjects, Box::getBoundingBox(*_vertices), Direction::X,
                                                   PlaneSelectionAlgorithmFactory::create(_algorithm), nodeRegister, _particleMode);
    }

    std::shared_ptr<TreeNode> KDTree::getRootNode() {
        LOG_DEBUG("KDTree: getRootNode called");
        if (!this->_rootNodeCreated.load(std::memory_order_acquire)) {
            std::lock_guard lock(this->_rootNodeCreationMutex);
            if (!this->_rootNodeCreated.load(std::memory_order_relaxed)) {
                LOG_DEBUG("KDTree: Root node is null, creating root node");
                this->_rootNode = TreeNodeFactory::createTreeNode(*_splitParam, 0);
                this->_rootNodeCreated.store(true, std::memory_order_release);
            } else {
                LOG_DEBUG("KDTree: Root node already created by another thread in the meantime, returning existing node");
            }
        } else {
            LOG_DEBUG("KDTree: Root node already created, returning existing node");
        }
        return this->_rootNode;
    }

    size_t KDTree::countIntersections(const Vertex &origin, const Vertex &ray) {
        //it's possible that a single intersection point is on the edge between two shapes. The point would be counted twice if the intersection points were not documented -> use of std::set
        LOG_DEBUG("KDTree: Counting intersections");
        //it's possible that a single intersection point is on the edge between two triangles. The point would be counted twice if the intersection points were not documented -> use of std::set
        std::set<Vertex> set{};
        this->getIntersections(origin, ray, set);
        LOG_DEBUG("KDTree: Intersections counted: " + std::to_string(set.size()));
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
        LOG_DEBUG("KDTree: getIntersections finished");
    }

#if defined(KD_TREE_OMP) or defined(KD_TREE_CPP)

    KDTree &KDTree::prebuildTree() {
        LOG_DEBUG("KDTree: prebuildTree called");
        //queue for children of processed nodes
#pragma omp parallel
        {
#pragma omp single
            {
                buildChildNodes(getRootNode());
            }
        }
        LOG_DEBUG("KDTree: prebuildTree finished");
        return *this;
    }
#endif

#if defined(KD_TREE_TBB)

    KDTree &KDTree::prebuildTree() {
        LOG_INFO("KDTree: prebuildTree called");
        tbb::task_group tasks;
        buildChildNodes(getRootNode(), tasks);
        tasks.wait();
        LOG_INFO("KDTree: prebuildTree finished");
        return *this;
    }

#endif

    std::pair<std::vector<GeometryObject>::iterator, std::vector<GeometryObject>::iterator>
    KDTree::geometryIterator() {
        return {_geometryObjects.begin(), _geometryObjects.end()};
    }

    std::pair<PlaneIterator, PlaneIterator> KDTree::planeIterator() {
        PlaneIterator begin{};
        std::shared_ptr<SplitNode> splitNode;
        // check whether the root node exists and if it's a SplitNode -> only then can there be planes to be iterated over
        if (getRootNode() != nullptr && (splitNode = std::dynamic_pointer_cast<SplitNode>(getRootNode())) != nullptr) {
            begin = PlaneIterator(splitNode);
        }
        return {begin, PlaneIterator{}};
    }

    bool KDTree::rebuildTreeIfNeeded() {
        std::shared_lock lock(nodeRegister.leafNodeMutex);
        const bool rebuildTree = std::ranges::any_of(nodeRegister.leafNodes.cbegin(), nodeRegister.leafNodes.cend(), [](const auto &leafNodePtr) {
            if (const auto leafNode = leafNodePtr.lock()) {
                if (leafNode->needTreeRebuild()) {
                    return true;
                }
            }
            return false;
        });
        lock.unlock();
        if (!rebuildTree) {
            return false;
        }
        LOG_DEBUG("KDTree: Rebuild needed due to vertex movement outside leaf node bounding box. Rebuilding tree...");
        this->rebuildTree();
        return true;
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
