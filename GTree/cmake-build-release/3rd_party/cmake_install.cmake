# Install script for directory: C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/3rd_party

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/Easy3D")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/glew/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/glfw/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/imgui/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/rply/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/lastools/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/kdtree/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/ransac/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/poisson/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/triangle/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/tetgen/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/glutess/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/backward/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/easyloggingpp/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/kd_tree/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/optimizer_lm/cmake_install.cmake")
  include("C:/Users/noort/Documents/1/TU/year_5/GEO2020_thesis/Code/Thesis_treegrowth/treegrowthmodelling/GTree/cmake-build-release/3rd_party/cminpack/cmake_install.cmake")

endif()

