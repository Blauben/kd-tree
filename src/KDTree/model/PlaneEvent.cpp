#include "KDTree/model/PlaneEvent.h"

namespace kdtree {

    PlaneEvent::PlaneEvent(const PlaneEventType type, const Plane plane, const unsigned objIndex)
        : type{type}, plane{plane}, objIndex{objIndex} {
    }

    bool PlaneEvent::operator<(const PlaneEvent &other) const {
        if (this->plane.axisCoordinate != other.plane.axisCoordinate) {
            return this->plane.axisCoordinate < other.plane.axisCoordinate;
        }
        if (this->plane.orientation == other.plane.orientation) {
            return this->type < other.type;
        }
        return static_cast<unsigned>(this->plane.orientation) < static_cast<unsigned>(other.plane.orientation);
    }

    bool PlaneEvent::operator==(const PlaneEvent &other) const {
        return type == other.type && plane == other.plane && objIndex == other.objIndex;
    }

    ObjectIndexVector convertEventsToGeometry(const std::variant<ObjectIndexVector, PlaneEventVector> &events) {
        if (std::holds_alternative<ObjectIndexVector>(events)) {
            return std::get<ObjectIndexVector>(events);
        }
        const auto &eventList{std::get<PlaneEventVector>(events)};
        ObjectIndexVector shapes{};
        shapes.reserve(eventList.size());
        //used to avoid duplication
        std::unordered_set<size_t> processedGeometry{};
        auto insertIfAbsent = [&shapes, &processedGeometry](const auto &planeEvent) {
            const auto geometryIndex{planeEvent.objIndex};
            if (!processedGeometry.contains(geometryIndex)) {
                processedGeometry.insert(geometryIndex);
                shapes.push_back(geometryIndex);
            }
        };
        std::ranges::for_each(eventList, insertIfAbsent);
        shapes.shrink_to_fit();
        return shapes;
    }

    size_t countGeometryObjects(const std::variant<ObjectIndexVector, PlaneEventVector> &geometry) {
        return std::visit(util::overloaded{
                                  [](const ObjectIndexVector &indexList) {
                                      return indexList.size();
                                  },
                                  [](const PlaneEventVector &eventList) {
                                      size_t count{0};
                                      std::unordered_set<size_t> processedGeometry{};
                                      std::ranges::for_each(eventList,
                                                            [&processedGeometry, &count](const auto &planeEvent) {
                                                                if (!processedGeometry.contains(planeEvent.objIndex)) {
                                                                    processedGeometry.insert(planeEvent.objIndex);
                                                                    count++;
                                                                }
                                                            });
                                      return count;
                                  }},
                          geometry);
    }
}// namespace kdtree
