cmake_minimum_required(VERSION 3.10.0)

project(discrete-curvature)

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Print the build type
# set(CMAKE_BUILD_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release" FORCE)
endif()
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

# Edit files to build here.
set(SRCS 
  src/main.cpp
  src/colormap.cpp
)

add_subdirectory(../../deps/geometry-central deps/geometry-central)
add_subdirectory(../../deps/polyscope deps/polyscope)

add_executable(main "${SRCS}")
target_include_directories(main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../deps/polyscope/deps/args")
target_link_libraries(main geometry-central polyscope)