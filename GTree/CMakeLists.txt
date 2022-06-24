cmake_minimum_required(VERSION 3.12)

if (APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.12" CACHE STRING "Minimum OS X deployment version" FORCE)
    message(STATUS "Minimum OS X deployment version: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif ()

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})
#project(Easy3D)

################################################################################

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

################################################################################

# Detects whether this is a top-level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(GTREE_TOPLEVEL_PROJECT ON)
else()
    set(GTREE_TOPLEVEL_PROJECT OFF)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)


################################################################################

### Configuration
set(GTREE_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(GTREE_THIRD_PARTY ${GTREE_ROOT}/3rd_party)

set(EASY3D_INCLUDE_DIR ${GTREE_ROOT})
set(EASY3D_SOURCE_DIR ${GTREE_ROOT})
set(EASY3D_BINARY_DIR ${CMAKE_BINARY_DIR})

set(GTREE_INCLUDE_DIR ${GTREE_ROOT})
set(GTREE_SOURCE_DIR ${GTREE_ROOT})
set(GTREE_BINARY_DIR ${CMAKE_BINARY_DIR})

### additional library not part of easy3d
set(GTREE_cminpack_INCLUDE_DIR "${GTREE_THIRD_PARTY}/cminpack")
set(GTREE_kd_INCLUDE_DIR "${GTREE_THIRD_PARTY}/kd_tree")

### libraries for growth interpolation
set(GTREE_glew_INCLUDE_DIR "${GTREE_EXTERNAL}/glew/include")
set(GTREE_glfw_INCLUDE_DIR "${GTREE_EXTERNAL}/glfw/include")
set(GTREE_imgui_INCLUDE_DIR "${GTREE_EXTERNAL}/imgui")
set(GTREE_tetgen_INCLUDE_DIR "${GTREE_EXTERNAL}/tetgen")
set(GTREE_easy3d_INCLUDE_DIR "${GTREE_EXTERNAL}" "${GTREE_ROOT}")
set(GTREE_lm_INCLUDE_DIR "${GTREE_EXTERNAL}/optimizer_lm")

### conditionally compile certain modules depending on libraries found on the system
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

################################################################################

add_subdirectory(3rd_party)

add_subdirectory(easy3d)

add_subdirectory(Interpolation)

add_subdirectory(TreeGrowthReconstruction)

################################################################################

# hide some variables that might be set in 3rd_party libraries
mark_as_advanced(FORCE BUILD_SHARED_LIBS)

################################################################################
