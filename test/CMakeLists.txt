# Includes the gtest.cmake
include(gtest)

# includes indicators.cmake for progress bars in tests
include(indicators)

# Recursively collects all the test files in the test Directory
file(GLOB_RECURSE TEST_SRC
        "${KD_TREE_SOURCE_DIR}/test/*.cpp"
        "${KD_TREE_SOURCE_DIR}/tests/*.h"
)

# Creates the Test Target
add_executable(${PROJECT_NAME}_test ${TEST_SRC})
# Builds the meshes used for the tests via script/scale_mesh.py (see cmake/scale_test_meshes.cmake)
add_dependencies(${PROJECT_NAME}_test scale_test_meshes)

set_target_properties(${PROJECT_NAME}_test PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
)

# Links target breakupModelTest against gtest_main (entry point, so no own main method is required)
target_link_libraries(${PROJECT_NAME}_test
        indicators
        GTest::gtest_main
        GTest::gtest
        GTest::gmock
        ${PROJECT_NAME}_lib
)
kd_tree_copy_runtime_dlls(${PROJECT_NAME}_test)

target_include_directories(${PROJECT_NAME}_test PUBLIC ${KD_TREE_SOURCE_DIR}/src)

include(GoogleTest)

# Adds the Tests to CTest by querying the test target executable
gtest_discover_tests(${PROJECT_NAME}_test
        WORKING_DIRECTORY ${KD_TREE_BINARY_DIR}
        DISCOVERY_TIMEOUT 120
)
