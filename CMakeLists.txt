# References for CMake:
# - The official tutorial: https://cmake.org/cmake/help/latest/guide/tutorial/index.html
# - Styles: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1

# This may not be very strict
cmake_minimum_required(VERSION 3.10)

project(macposts)
set(CMAKE_CXX_STANDARD 11)

## Main library

set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Build Eigen .pc file" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Build Eigen documentation" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build Eigen tests" FORCE)
add_subdirectory(lib/eigen)

add_subdirectory(lib/snap)
set_property(TARGET snap PROPERTY POSITION_INDEPENDENT_CODE ON)

# NOTE: Do not use glob here. Always list source files explicitly. See the style
# guide at the top of this file.
add_library(macposts STATIC
  src/cc_tree.cpp
  src/due.cpp
  src/pre_routing.cpp
  src/statistics.cpp
  src/dlink.cpp
  src/emission.cpp
  src/marginal_cost.cpp
  src/realtime_dta.cpp
  src/ults.cpp
  src/dnode.cpp
  src/factory.cpp
  src/multiclass.cpp
  src/routing.cpp
  src/vehicle.cpp
  src/dta.cpp
  src/gridlock_checker.cpp
  src/od.cpp
  src/shortest_path.cpp
  src/vms.cpp
  src/dta_gradient_utls.cpp
  src/io.cpp
  src/path.cpp
  src/so_routing.cpp
  src/workzone.cpp
  )
target_link_libraries(macposts
  # TODO: Some header files in src expose Eigen. Check if they can be eliminated
  # and maybe make this private.
  PUBLIC Eigen3::Eigen
  # We expose many Snap data structures and functions in the interface so this
  # needs to be public.
  PUBLIC snap
  )
target_include_directories(macposts INTERFACE src)
set_property(TARGET macposts PROPERTY POSITION_INDEPENDENT_CODE ON)

## Python binding

add_subdirectory(lib/pybind11)
pybind11_add_module(_macposts_ext macposts/_macposts_ext.cpp)
target_link_libraries(_macposts_ext PRIVATE macposts)
