#include "KDTree/plane_selection/PlaneEventAlgorithm.h"

namespace kdtree {
    ShapeCounter::ShapeCounter(const size_t dimensionCount, const std::array<size_t, 3> &initialValues)
        : _dimensionShapeValues(dimensionCount, initialValues) {
        if (dimensionCount == 0) {
            throw std::invalid_argument("Dimension count must be greater than zero");
        }
    }

    void ShapeCounter::updateMax(Direction direction, const size_t p_planar, const size_t p_end) {
        _dimensionShapeValues.at(static_cast<size_t>(direction) % _dimensionShapeValues.size()).at(1) -= p_planar +
                                                                                                         p_end;
    }

    void ShapeCounter::updateMin(Direction direction, const size_t p_planar, const size_t p_start) {
        _dimensionShapeValues.at(static_cast<size_t>(direction) % _dimensionShapeValues.size()).at(0) += p_planar +
                                                                                                         p_start;
    }

    void ShapeCounter::setPlanar(Direction direction, const size_t p_planar) {
        _dimensionShapeValues.at(static_cast<size_t>(direction) % _dimensionShapeValues.size()).at(2) = p_planar;
    }

    size_t ShapeCounter::getMin(Direction direction) const {
        return _dimensionShapeValues.at(static_cast<size_t>(direction) % _dimensionShapeValues.size()).at(0);
    }

    size_t ShapeCounter::getMax(Direction direction) const {
        return _dimensionShapeValues.at(static_cast<size_t>(direction) % _dimensionShapeValues.size()).at(1);
    }

    size_t ShapeCounter::getPlanar(Direction direction) const {
        return _dimensionShapeValues.at(static_cast<size_t>(direction) % _dimensionShapeValues.size()).at(2);
    }

    PlaneEventVector PlaneEventAlgorithm::generatePlaneEventsFromGeometry(const SplitParam &splitParam,
                                                                          std::vector<Direction> directions) {
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundObjects)) {
            return std::get<PlaneEventVector>(splitParam.boundObjects);
        }
        const auto &boundShapes{std::get<ObjectIndexVector>(splitParam.boundObjects)};
        //transform the shapes into vertices
        auto [vertex_begin, vertex_end] = transformIterator(boundShapes.cbegin(), boundShapes.cend(),
                                                            splitParam.geometryObjects);

        // Store the clipped results in a struct to avoid multiple passes over the data
        struct ClipResult {
            Vertex minPoint;
            Vertex maxPoint;
            size_t objIndex;
            std::array<bool, 3> isPlanarInDirection;
        };

        // Generate clipped results for each shape in parallel using thrust
        thrust::host_vector<ClipResult> clippedResults(boundShapes.size());
        thrust::transform(thrust::host, vertex_begin, vertex_end, clippedResults.begin(),
                          [&splitParam, &directions](const auto &indexAndVertices) {
                              const auto [index, vertices] = indexAndVertices;
                              const auto clippedVertices = splitParam.boundingBox.clipToVoxel(vertices);
                              const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Vertex>>(clippedVertices);
                              std::array<bool, 3> isPlanarInDirection{};
                              for (const auto direction: directions) {
                                  isPlanarInDirection[static_cast<size_t>(direction)] = minPoint[static_cast<size_t>(direction)] == maxPoint[static_cast<size_t>(direction)];
                              }
                              return ClipResult{minPoint, maxPoint, index, isPlanarInDirection};
                          });

        // Generate plane events from the clipped results
        // 1. Extract counts (how many events each shape will generate) — respecting `directions`
        thrust::host_vector<int> counts(boundShapes.size());
        thrust::transform(clippedResults.begin(), clippedResults.end(), counts.begin(),
                          [&directions](const ClipResult &s) {
                              int total = 0;
                              for (const auto &dir: directions) {
                                  total += s.isPlanarInDirection[static_cast<int>(dir)] ? 1 : 2;
                              }
                              return total;
                          });

        // 2. create a vector of offsets for each shape's events in the output vector
        thrust::host_vector<int> offsets(boundShapes.size());
        thrust::exclusive_scan(counts.begin(), counts.end(), offsets.begin());
        size_t total = offsets.empty() ? 0 : offsets.back() + counts.back();

        // 3. Scatter the input index at each group's start offset
        thrust::host_vector<int> input_index_map(total, 0);
        thrust::scatter_if(
                thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(boundShapes.size()),// values = 0..n-1
                offsets.begin(),                                                                      // where to scatter each value
                counts.begin(),                                                                       // stencil
                input_index_map.begin(),
                [](int c) { return c > 0; });// only scatter for non-empty groups

        // Fill the gaps: inclusive scan with "max" propagates each index forward
        thrust::inclusive_scan(
                input_index_map.begin(), input_index_map.end(),
                input_index_map.begin(),
                thrust::maximum<int>());

        // 4. local index within group (internal numbering of the events for each shape)
        thrust::host_vector<int> local_index(total);
        thrust::transform(
                thrust::counting_iterator<int>(0), thrust::counting_iterator<int>((int) total),
                thrust::make_permutation_iterator(offsets.begin(), input_index_map.begin()),
                local_index.begin(), thrust::minus<int>());

        std::vector<PlaneEvent> output(total);
        thrust::transform(
                thrust::make_permutation_iterator(clippedResults.begin(), input_index_map.begin()),
                thrust::make_permutation_iterator(clippedResults.begin(), input_index_map.begin()) + total,
                local_index.begin(),
                output.begin(),
                [&directions](const ClipResult &src, int localIdx) {
                    for (const auto &dir: directions) {
                        const size_t i = static_cast<size_t>(dir);
                        if (src.isPlanarInDirection[i]) {
                            if (localIdx == 0) {
                                return PlaneEvent(PlaneEventType::planar, Plane(src.minPoint, dir), src.objIndex);
                            }
                            localIdx--;
                        } else {
                            if (localIdx == 0) {
                                return PlaneEvent(PlaneEventType::starting, Plane(src.minPoint, dir), src.objIndex);
                            } else if (localIdx == 1) {
                                return PlaneEvent(PlaneEventType::ending, Plane(src.maxPoint, dir), src.objIndex);
                            }
                            localIdx -= 2;
                        }
                    }
                    throw std::runtime_error("PlaneEventAlgorithm::generatePlaneEventsFromGeometry: Invalid local index for generating PlaneEvent");
                });

        std::sort(output.begin(), output.end());
        return output;
    }

}// namespace kdtree
