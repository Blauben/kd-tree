nanobind_add_module(KDTree_Python 
    STABLE_ABI
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


add_custom_command(TARGET KDTree_Python POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "KDTree_Python built at: $<TARGET_FILE:KDTree_Python>"
)
