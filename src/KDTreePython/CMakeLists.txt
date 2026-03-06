nanobind_add_module(kdtree_py
    KDTreePython.cpp
)

target_link_libraries(kdtree_py PUBLIC ${PROJECT_NAME}_lib ${Python_LIBRARIES})

target_include_directories(kdtree_py PUBLIC ${CMAKE_SOURCE_DIR}/src)


# Install the kdtree_py module and __init__.py into the correct Python package directory
if (INSTALL_KD_TREE_PYTHON_INTERFACE)
    install(TARGETS kdtree_py
        LIBRARY DESTINATION kdtree_py
        COMPONENT python_interface
    )
    install(FILES __init__.py
        DESTINATION kdtree_py
        COMPONENT python_interface
    )
endif()

# Place the executable in the top level directory
set_target_properties(kdtree_py PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
        LIBRARY_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
)


add_custom_command(TARGET kdtree_py POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "kdtree_py built at: $<TARGET_FILE:kdtree_py>"
)
