cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

# add every new class file here!
set(${PROJECT_NAME}_SOURCES
        main.cpp
        graph_gt.h
        growth_viewer_imgui.cpp
        growth_viewer_imgui.h
        growth_viewer.cpp
        growth_viewer.h
        g_tree.cpp
        g_tree.h
        lobe.cpp
        lobe.h
        skeleton_grow.cpp
        skeleton_grow.h
        growth_model_region.cpp
        growth_model_region.h
        cylinder_fitter.cpp
        cylinder_fitter.h
        cylinder.h
        )


if (APPLE)
    function(resource VAR SOURCE_PATH DESTINATION PATTERN)
        file(GLOB_RECURSE _LIST CONFIGURE_DEPENDS ${SOURCE_PATH}/${PATTERN})
        foreach (RESOURCE ${_LIST})
            get_filename_component(_PARENT ${RESOURCE} DIRECTORY)
            if (${_PARENT} STREQUAL ${SOURCE_PATH})
                set(_DESTINATION ${DESTINATION})
            else ()
                file(RELATIVE_PATH _DESTINATION ${SOURCE_PATH} ${_PARENT})
                set(_DESTINATION ${DESTINATION}/${_DESTINATION})
            endif ()
            set_property(SOURCE ${RESOURCE} PROPERTY MACOSX_PACKAGE_LOCATION ${_DESTINATION})
        endforeach (RESOURCE)
        set(${VAR} ${_LIST} PARENT_SCOPE)
    endfunction()

    resource(VERTEX_SHADERS ${GTREE_ROOT}/resources/shaders Resources/shaders *.vert)
    resource(FRAGMENT_SHADERS ${GTREE_ROOT}/resources/shaders Resources/shaders *.frag)

    # todo: find different icon/rename for gtree
    # This tells CMake where to find and install the ICON file
    set(${PROJECT_NAME}_ICON ${GTREE_ROOT}/resources/icons/adtree.icns)

    #Append the new files to the resources file list
    list(APPEND ${PROJECT_NAME}_SOURCES ${${PROJECT_NAME}_ICON} ${VERTEX_SHADERS} ${FRAGMENT_SHADERS})

    add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${${PROJECT_NAME}_ICON} ${${PROJECT_NAME}_SOURCES})

    set_source_files_properties(${${PROJECT_NAME}_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

    ## Note: On macOS, high-DPI support is enabled by settings in the Info.plist file. Make sure they are present.
    ## For more information, see: http://doc.qt.io/qt-5/highdpi.html
    ## Enable high DPI support by using my Info.plist file in which I added NSHighResolutionCapable to be true.
    set_target_properties(${PROJECT_NAME} PROPERTIES
            INSTALL_RPATH "@executable_path/../Frameworks"
            MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_INFO_PLIST "${GTREE_ROOT}/resources/Mac/adtree.plist"
            MACOSX_BUNDLE_ICON_FILE "adtree.icns"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0"
            MACOSX_BUNDLE_LONG_VERSION_STRING "1.0"
            MACOSX_BUNDLE_BUNDLE_VERSION "1.0"
            MACOSX_BUNDLE_GUI_IDENTIFIER "liangliang.nan@gmail.com"
            )
else ()
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
endif ()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "TreeGrowthReconstruction")

target_include_directories(${PROJECT_NAME} PRIVATE ${GTREE_glew_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GTREE_glfw_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GTREE_easy3d_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GTREE_tetgen_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GTREE_kd_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GTREE_lm_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)

### required packages
find_package(OpenGL REQUIRED)
message(STATUS "OpenGL libraries: ${OPENGL_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} 3rd_glew 3rd_glfw 3rd_imgui easy3d::core easy3d::renderer easy3d::util easy3d::fileio easy3d::algo easy3d::viewer 3rd_tetgen 3rd_cminpack 3rd_optimizer_lm 3rd_kdtree)


find_package(Boost REQUIRED) # It's "Boost", not "BOOST" or "boost". Case matters.
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS} ${Boost_headers_DIR})

# tell the idiot program to actually link to the CGAL libraries
find_package(CGAL REQUIRED)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
###