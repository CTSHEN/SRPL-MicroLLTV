cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(srpl_micro_lltv_gazebo_plugin)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    #gazebo_dev
    roscpp
    srpl_micro_lltv
    )
find_package(Eigen3 REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS srpl_micro_lltv
)



include_directories(${PROJECT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_MSG_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

#link_directories(
#  ${catkin_LIBRARY_DIRS}
#  ${EIGEN3_LIBRARY_DIRS})
  
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED model_push/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(model_force SHARED model_push/model_force.cc)
target_link_libraries(model_force ${GAZEBO_LIBRARIES})

add_library(srpl_mass_balance srpl_micro_lltv/srpl_mass_balance.cpp)
target_link_libraries(srpl_mass_balance ${GAZEBO_LIBRARIES})

add_library(srpl_thruster SHARED srpl_micro_lltv/MonoThrusterPlugin.cc)
add_dependencies(srpl_thruster srpl_micro_lltv_generate_messages_cpp)
target_include_directories(srpl_thruster PUBLIC srpl_micro_lltv/include)
target_link_libraries(srpl_thruster ${GAZEBO_LIBRARIES})