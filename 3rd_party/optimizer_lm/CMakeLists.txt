cmake_minimum_required(VERSION 3.1)

project(3rd_optimizer_lm)


add_library(3rd_optimizer_lm STATIC 
	optimizer_lm.h
	optimizer_lm.cpp
)

set_target_properties(3rd_optimizer_lm PROPERTIES FOLDER "3rd_party")


target_include_directories(3rd_optimizer_lm PRIVATE ${GTREE_cminpack_INCLUDE_DIR})

target_link_libraries(3rd_optimizer_lm 3rd_cminpack)
