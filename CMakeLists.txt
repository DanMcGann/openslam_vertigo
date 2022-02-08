project(vertigo CXX C)
cmake_minimum_required(VERSION 2.8.3)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# Vertigo GTSAM Switchable constraints library
add_library(vertigo INTERFACE)
target_link_libraries(vertigo INTERFACE gtsam)
target_include_directories(vertigo INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")