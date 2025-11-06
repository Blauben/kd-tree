# Including the actual Polyhedral source files
file(GLOB_RECURSE SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(${PROJECT_NAME}_lib OBJECT ${SRC} Info.h
        model/GeometryObject.h
        model/Point.h
        model/Triangle.h
        model/GeometryObject.cpp)
