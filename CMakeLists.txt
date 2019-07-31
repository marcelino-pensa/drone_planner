cmake_minimum_required(VERSION 2.8.3)
project(drone_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  cmake_modules
  geometry_msgs
  mavros_msgs
  message_generation
  roscpp
  std_msgs
  # p4_ros
)

find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg FILES
   PVA_4d.msg
)

add_service_files( FILES
  trapezoidal_p2p.srv
  publish_pose.srv
)

add_action_files(DIRECTORY action 
                 FILES trapezoidal_p2p.action)
generate_messages(DEPENDENCIES 
  actionlib_msgs geometry_msgs)

catkin_package(
  DEPENDS Eigen 
  # p4_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(drone_planner_node src/drone_planner.cpp
                                  src/services.cpp
                                  src/trapezoidal_p2p_action.cpp
                                  src/trapezoidal.cpp
                                  src/visualization_functions.cpp
                                  src/pevents.cpp)
target_link_libraries(drone_planner_node ${catkin_LIBRARIES})

#This makes sure that messages and services are compiled before the rest
add_dependencies(drone_planner_node drone_planner_generate_messages_cpp)
add_dependencies(drone_planner_node ${catkin_EXPORTED_TARGETS})
# add_dependencies(drone_planner_node p4_ros_gencpp)

# Compile client example
add_executable(drone_planner_client src/drone_planner_client_example.cpp)
target_link_libraries(drone_planner_client ${catkin_LIBRARIES})
add_dependencies(drone_planner_client drone_planner_generate_messages_cpp)
add_dependencies(drone_planner_client ${catkin_EXPORTED_TARGETS})
