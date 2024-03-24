# References for CMake:
# - The official tutorial: https://cmake.org/cmake/help/latest/guide/tutorial/index.html
# - Styles: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1

# This may not be very strict
cmake_minimum_required(VERSION 3.10)

project(macposts)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(macposts_warning_flags INTERFACE)
# GCC and Clang
set(MACPOSTS_GCC_WARNING_FLAGS
  -Wall -Wextra -Wpedantic
  # There are too many, and they are mostly benign. We will fix them later™.
  -Wno-unused-parameter
)
# MSVC
set(MACPOSTS_MSVC_WARNING_FLAGS
  -W4
  # `-Wno-unused-parameter'
  -wd4100
)
target_compile_options(macposts_warning_flags INTERFACE
  "$<$<CXX_COMPILER_ID:MSVC>:${MACPOSTS_MSVC_WARNING_FLAGS}>"
  "$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:${MACPOSTS_GCC_WARNING_FLAGS}>"
)

## Main library

set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Build Eigen .pc file" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Build Eigen documentation" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build Eigen tests" FORCE)
add_subdirectory(lib/eigen)

# NOTE: Do not use glob here. Always list source files explicitly. See the style
# guide at the top of this file.
add_library(macposts STATIC
  src/cc_tree.cpp
  src/delivery_traffic.cpp
  src/dlink.cpp
  src/dnode.cpp
  src/dso.cpp
  src/dta.cpp
  src/dta_gradient_utls.cpp
  src/due.cpp
  src/emission.cpp
  src/ev_traffic.cpp
  src/factory.cpp
  src/gridlock_checker.cpp
  src/io.cpp
  src/marginal_cost.cpp
  src/multiclass.cpp
  src/multimodal.cpp
  src/od.cpp
  src/path.cpp
  src/pre_routing.cpp
  src/realtime_dta.cpp
  src/routing.cpp
  src/shortest_path.cpp
  src/so_routing.cpp
  src/statistics.cpp
  src/ults.cpp
  src/vehicle.cpp
  src/vms.cpp
  src/workzone.cpp
)
target_link_libraries(macposts
  # TODO: Some header files in src expose Eigen. Check if they can be eliminated
  # and maybe make this private.
  PUBLIC Eigen3::Eigen
  PRIVATE macposts_warning_flags
)
target_include_directories(macposts INTERFACE src)
set_property(TARGET macposts PROPERTY POSITION_INDEPENDENT_CODE ON)

## Python binding

add_subdirectory(lib/pybind11)
pybind11_add_module(_macposts_ext
  macposts/_ext/dta.cpp
  macposts/_ext/macposts.cpp
  macposts/_ext/mcdta.cpp
  macposts/_ext/mmdta.cpp
  macposts/_ext/tdsp.cpp
)
target_link_libraries(_macposts_ext PRIVATE macposts macposts_warning_flags)
