cmake_minimum_required(VERSION 2.8.3)
project(drone_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

option(COMPILE_MIN_TIME_SERVICE "Compile minimum time service (requires OSQP and ECOS)" FALSE) # TRUE/FALSE


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
)

find_package(Eigen REQUIRED)

# Find OSQP in case the min time service needs to be compiled
if(COMPILE_MIN_TIME_SERVICE)
  find_package(osqp REQUIRED)
endif(COMPILE_MIN_TIME_SERVICE)

add_message_files(DIRECTORY msg FILES
   PVA.msg
   PVA_4d.msg
)

add_service_files( FILES
  trapezoidal_p2p.srv
  min_time.srv
  publish_pose.srv
)

add_action_files(DIRECTORY action 
                 FILES trapezoidal_p2p.action)
generate_messages(DEPENDENCIES 
  actionlib_msgs geometry_msgs)

catkin_package(
  DEPENDS Eigen
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

if(COMPILE_MIN_TIME_SERVICE)
  # Extra directories to include/link in case we need to compile the 
  # min time service
  include_directories(
    dependencies/P4/src
    dependencies/TimeOptimizer/src
    dependencies/ecos/include
    dependencies/ecos/external/ldl/include
    dependencies/ecos/external/SuiteSparse_config
  )
  link_directories( ${PROJECT_SOURCE_DIR}/dependencies/ecos
                    ${PROJECT_SOURCE_DIR}/dependencies/ecos/external/ldl)

  add_executable(drone_planner_node src/drone_planner.cpp
                                    src/services_updated.cpp
                                    src/trapezoidal_p2p_action.cpp
                                    src/trapezoidal.cpp
                                    src/visualization_functions.cpp
                                    src/geometry_functions.cpp
                                    src/helper.cpp
                                    src/p4_helper.cpp
                                    src/time_optimizer_class.cpp
                                    dependencies/P4/src/polynomial_solver.cc
                                    dependencies/P4/src/polynomial_sampler.cc
                                    dependencies/TimeOptimizer/src/make_sparse.cpp
                                    dependencies/TimeOptimizer/src/time_optimizer_ecos.cpp
                                    src/pevents.cpp)
  target_link_libraries(drone_planner_node osqp::osqp libldl.a libecos.a)
else()
  add_executable(drone_planner_node src/drone_planner.cpp
                                    src/services.cpp
                                    src/trapezoidal_p2p_action.cpp
                                    src/trapezoidal.cpp
                                    src/visualization_functions.cpp
                                    src/geometry_functions.cpp
                                    src/helper.cpp
                                    src/pevents.cpp)
endif(COMPILE_MIN_TIME_SERVICE)

target_link_libraries(drone_planner_node ${catkin_LIBRARIES})


#This makes sure that messages and services are compiled before the rest
add_dependencies(drone_planner_node drone_planner_generate_messages_cpp)
add_dependencies(drone_planner_node ${catkin_EXPORTED_TARGETS})

# Compile trapezoidal client example
add_executable(trapezoidal_client examples/trapezoidal_client_example.cpp)
target_link_libraries(trapezoidal_client ${catkin_LIBRARIES})
add_dependencies(trapezoidal_client drone_planner_generate_messages_cpp)
add_dependencies(trapezoidal_client ${catkin_EXPORTED_TARGETS})

# Compile min time client example
add_executable(min_time_client examples/min_time_client_example.cpp
                               src/helper.cpp)
target_link_libraries(min_time_client ${catkin_LIBRARIES})
add_dependencies(min_time_client drone_planner_generate_messages_cpp)
add_dependencies(min_time_client ${catkin_EXPORTED_TARGETS})

unset(COMPILE_MIN_TIME_SERVICE CACHE)