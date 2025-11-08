//
// Created by saruman on 07.11.25.
//
#include "KDTree/model/PlaneEvent.h"

namespace kdtree {

    PlaneEvent::PlaneEvent(const PlaneEventType type, const Plane plane, const unsigned faceIndex)
        : type{type}, plane{plane}, faceIndex{faceIndex} {
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
        return type == other.type && plane == other.plane && faceIndex == other.faceIndex;
    }

    ObjectIndexVector convertEventsToGeometry(const std::variant<ObjectIndexVector, PlaneEventVector> &events) {
        if (std::holds_alternative<ObjectIndexVector>(events)) {
            return std::get<ObjectIndexVector>(events);
        }
        const auto &eventList{std::get<PlaneEventVector>(events)};
        ObjectIndexVector triangles{};
        triangles.reserve(eventList.size());
        //used to avoid duplication
        std::unordered_set<size_t> processedFaces{};
        auto insertIfAbsent = [&triangles, &processedFaces](const auto &planeEvent) {
            const auto faceIndex{planeEvent.faceIndex};
            if (processedFaces.find(faceIndex) == processedFaces.end()) {
                processedFaces.insert(faceIndex);
                triangles.push_back(faceIndex);
            }
        };
        std::for_each(eventList.cbegin(), eventList.cend(), insertIfAbsent);
        triangles.shrink_to_fit();
        return triangles;
    }

    size_t countFaces(const std::variant<ObjectIndexVector, PlaneEventVector> &triangles) {
        return std::visit(util::overloaded{
                                  [](const ObjectIndexVector &indexList) {
                                      return indexList.size();
                                  },
                                  [](const PlaneEventVector &eventList) {
                                      size_t count{0};
                                      std::unordered_set<size_t> processedFaces{};
                                      std::for_each(eventList.cbegin(), eventList.cend(),
                                                    [&processedFaces, &count](const auto &planeEvent) {
                                                        if (processedFaces.find(planeEvent.faceIndex) == processedFaces.end()) {
                                                            processedFaces.insert(planeEvent.faceIndex);
                                                            count++;
                                                        }
                                                    });
                                      return count;
                                  }},
                          triangles);
    }
}