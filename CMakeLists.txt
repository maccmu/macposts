# References for CMake:
# - The official tutorial: https://cmake.org/cmake/help/latest/guide/tutorial/index.html
# - Styles: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1

# This may not be very strict
cmake_minimum_required(VERSION 3.10)

project(macposts VERSION 0.1.0)

## Dependencies

set(DEP_LIBS "")                # Libraries
set(DEP_INCS "")                # Include directories

# Eigen
set(CMAKE_CXX_STANDARD 14)      # Eigen requires C++ 14
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(lib/eigen)
list(APPEND DEP_LIBS PRIVATE Eigen3::Eigen)

# Snap
find_package(OpenMP REQUIRED)   # Snap requires OpenMP
list(APPEND DEP_LIBS PRIVATE OpenMP::OpenMP_CXX)

set(SNAP_DIR ${CMAKE_SOURCE_DIR}/lib/snap/snap-core)
set(GLIB_DIR ${CMAKE_SOURCE_DIR}/lib/snap/glib-core)
add_custom_command(
  OUTPUT ${SNAP_DIR}/libsnap.a
  COMMAND make -C ${SNAP_DIR} lib
  )
list(APPEND DEP_LIBS PRIVATE ${SNAP_DIR}/libsnap.a)
list(APPEND DEP_INCS PRIVATE ${SNAP_DIR})
list(APPEND DEP_INCS PRIVATE ${GLIB_DIR})

# Advds
#
# TODO: Remove this one. It is an assortment of some files from different
# sources. So it is hard to keep track with the upstream. Even worse, some files
# have severe issues regarding the license. So we should not use them. Luckily
# those seems easy to write from scratch or find an alternative from well-known
# libraries (or even the standard libraries).
list(APPEND DEP_INCS PRIVATE ${CMAKE_SOURCE_DIR}/lib/advds)

## Main library

# NOTE: Do not use glob here. Always list source files explicitly. See the style
# guide at the top of this file.
add_library(macposts STATIC
  src/cc_tree.cpp
  src/due.cpp
  src/k_shortest_path.cpp
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
target_include_directories(macposts INTERFACE src ${DEP_INCS})
target_link_libraries(macposts ${DEP_LIBS})
