find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

nanobind_add_module(my_ext KDTreePython.cpp)

target_link_libraries(my_ext PRIVATE ${Python_LIBRARIES})

add_custom_command(TARGET my_ext POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "my_ext built at: $<TARGET_FILE:my_ext>"
)