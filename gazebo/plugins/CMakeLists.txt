cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED model_push/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(model_force SHARED model_push/model_force.cc)
target_link_libraries(model_force ${GAZEBO_LIBRARIES})

add_library(srpl_mass_balance SHARED srpl_micro_lltv/srpl_mass_balance.cpp)
target_link_libraries(srpl_mass_balance ${GAZEBO_LIBRARIES})
