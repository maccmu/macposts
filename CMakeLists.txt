# References for CMake:
# - The official tutorial: https://cmake.org/cmake/help/latest/guide/tutorial/index.html
# - Styles: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1

# This may not be very strict
cmake_minimum_required(VERSION 3.10)

project(macposts)
set(CMAKE_CXX_STANDARD 11)

## Dependencies

# Eigen
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Build Eigen .pc file" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Build Eigen documentation" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build Eigen tests" FORCE)
add_subdirectory(lib/eigen)

# Snap
find_package(OpenMP REQUIRED)   # Snap requires OpenMP

# TODO: Convert Snap to a CMake based project for each configuration.
set(SNAP_DIR ${CMAKE_SOURCE_DIR}/lib/snap/snap-core)
set(GLIB_DIR ${CMAKE_SOURCE_DIR}/lib/snap/glib-core)
add_custom_command(
  OUTPUT ${SNAP_DIR}/Snap.o
  COMMAND CXXFLAGS=-fPIC make -C ${SNAP_DIR} Snap.o
  )

## Main library

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
  # TODO: Here we compile the whole Snap library in to avoid potential issues.
  # However, it does not look tasteful.
  ${SNAP_DIR}/Snap.o
  )
target_link_libraries(macposts
  # TODO: Some header files in src expose Eigen. Check if they can be eliminated
  # and maybe make this private.
  PUBLIC Eigen3::Eigen
  # Snap requires OpenMP
  #
  # NOTE: It may not always require OpenMP but often does. See the
  # 'lib/snap/Makefile.config' file for how it is built on different platforms.
  PRIVATE OpenMP::OpenMP_CXX
  )
target_include_directories(macposts
  INTERFACE src
  # TODO: Check if those two are really necessary to be public.
  PUBLIC ${SNAP_DIR}
  PUBLIC ${GLIB_DIR}
  )
set_property(TARGET macposts PROPERTY POSITION_INDEPENDENT_CODE ON)

## Python binding

add_subdirectory(lib/pybind11)
pybind11_add_module(_macposts_ext macposts/_macposts_ext.cpp)
target_link_libraries(_macposts_ext PRIVATE macposts)
