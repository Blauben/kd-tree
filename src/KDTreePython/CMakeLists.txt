nanobind_add_module(KDTree_Python
    KDTreePython.cpp
)

target_link_libraries(KDTree_Python PUBLIC ${PROJECT_NAME}_lib ${Python_LIBRARIES})

target_include_directories(KDTree_Python PUBLIC ${CMAKE_SOURCE_DIR}/src)


# Install the KDTree_Python module and __init__.py into the correct Python package directory
if (INSTALL_KD_TREE_PYTHON_INTERFACE)
    install(TARGETS KDTree_Python
        LIBRARY DESTINATION KDTree_Python
        COMPONENT python_interface
    )
    install(FILES __init__.py
        DESTINATION KDTree_Python
        COMPONENT python_interface
    )
endif()

set(KD_TREE_PYTHON_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}" CACHE PATH "Output directory for the KDTree_Python module")

# Place the executable in the top level directory
set_target_properties(KDTree_Python PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${KD_TREE_PYTHON_OUTPUT_DIRECTORY}
        LIBRARY_OUTPUT_DIRECTORY ${KD_TREE_PYTHON_OUTPUT_DIRECTORY}
        RUNTIME_OUTPUT_DIRECTORY ${KD_TREE_PYTHON_OUTPUT_DIRECTORY}
)


add_custom_command(TARGET KDTree_Python POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "KDTree_Python built at: $<TARGET_FILE:KDTree_Python>"
)
