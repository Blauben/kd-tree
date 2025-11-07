# Including the actual Polyhedral source files
file(GLOB_RECURSE SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(${PROJECT_NAME}_lib OBJECT ${SRC} Info.h
        model/PlaneEvent.h
        model/Plane.cpp
        model/Box.cpp
        model/PlaneEvent.cpp)
