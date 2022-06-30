cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

add_executable(${PROJECT_NAME}
        main.cpp
        viewer_front.h
        viewer_front.cpp
        viewer_back.cpp
        viewer_back.h
        interpolator.cpp
        interpolator.h
        graph_boost.h
        cylinder_fitter.cpp
        cylinder_fitter.h
        cylinder.h
        )

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Interpolation")

target_link_libraries(${PROJECT_NAME} 3rd_imgui easy3d::core easy3d::renderer easy3d::viewer easy3d::util cminpack optimizer_lm kd_tree)

find_package(Boost REQUIRED) # It's "Boost", not "BOOST" or "boost". Case matters.
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS} ${Boost_headers_DIR})
