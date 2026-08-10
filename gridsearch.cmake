file(GLOB_RECURSE KD_TREE_SRC_FILES "src/KDTree/*.cpp" "src/KDTree/*.h")
list(REMOVE_ITEM KD_TREE_SRC_FILES "${KD_TREE_SOURCE_DIR}/src/KDTree/util/Constants.h") # Exclude Constants.h from the source files list

message(STATUS "KD Tree Source Files: ${KD_TREE_SRC_FILES}")

file(MAKE_DIRECTORY "${KD_TREE_BINARY_DIR}/src/KDTree/util")
file(READ "${KD_TREE_SOURCE_DIR}/src/KDTree/util/Constants.h" KD_TREE_CONSTANTS_CONTENT)
string(REPLACE "constexpr " "inline " KD_TREE_CONSTANTS_LINES "${KD_TREE_CONSTANTS_CONTENT}")
file(WRITE "${KD_TREE_BINARY_DIR}/src/KDTree/util/Constants.h" "${KD_TREE_CONSTANTS_LINES}")
list(APPEND KD_TREE_SRC_FILES "${KD_TREE_BINARY_DIR}/src/KDTree/util/Constants.h") # Add the modified Constants.h to the source files list

file(MAKE_DIRECTORY "${KD_TREE_BINARY_DIR}/src/KDTree")
file(COPY_FILE "${KD_TREE_SOURCE_DIR}/src/KDTree/Info.h.in" "${KD_TREE_BINARY_DIR}/src/KDTree/Info.h")

add_executable(gridsearch src/gridsearch.cpp ${KD_TREE_SRC_FILES})

target_include_directories(gridsearch
            PUBLIC
            ${KD_TREE_BINARY_DIR}/src
            ${KD_TREE_SOURCE_DIR}/src
    )

target_link_libraries(gridsearch
            PUBLIC
            tetgen::tetgen
            spdlog::$<IF:$<AND:$<PLATFORM_ID:Darwin>,$<CXX_COMPILER_ID:GNU>>,spdlog_header_only,spdlog>
            thrust::thrust_kdtree
    )

file(COPY ${KD_TREE_SOURCE_DIR}/resources DESTINATION ${KD_TREE_BINARY_DIR})
