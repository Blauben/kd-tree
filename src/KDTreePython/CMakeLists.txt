nanobind_add_module(scikdtree
    KDTreePython.cpp
)

target_link_libraries(scikdtree PUBLIC ${PROJECT_NAME}_lib)

target_include_directories(scikdtree PUBLIC ${KD_TREE_SOURCE_DIR}/src)

set(KD_TREE_PYTHON_SOURCE_DIR "${KD_TREE_SOURCE_DIR}/src/KDTreePython")

file(GLOB_RECURSE PYTHON_FILES
        "${KD_TREE_PYTHON_SOURCE_DIR}/*.py")

# Install the scikdtree module and python files into the correct Python package directory
if (INSTALL_KD_TREE_PYTHON_INTERFACE)
    install(TARGETS scikdtree
        LIBRARY DESTINATION scikdtree
        COMPONENT python_interface
    )
    install(FILES ${PYTHON_FILES}
        DESTINATION scikdtree
        COMPONENT python_interface
    )
    install(FILES $<TARGET_RUNTIME_DLLS:scikdtree>
        DESTINATION scikdtree
        COMPONENT python_interface
    )
endif()

# Place the executable in the top level directory
set_target_properties(scikdtree PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
        LIBRARY_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
)

add_custom_command(TARGET scikdtree POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "scikdtree built at: $<TARGET_FILE:scikdtree>"
)
