find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

if (NOT PYTHON_FOUND)
    message(FATAL_ERROR "Python executable not found! Please install Python.")
else()
    message(STATUS "Python found at ${Python_EXECUTABLE}")

    nanobind_add_module(KDTree_Python KDTreePython.cpp)

    target_link_libraries(KDTree_Python PRIVATE ${Python_LIBRARIES})

    add_custom_command(TARGET KDTree_Python POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "KDTree_Python built at: $<TARGET_FILE:KDTree_Python>"
    )

endif ()