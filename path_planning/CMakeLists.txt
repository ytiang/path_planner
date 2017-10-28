cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)
find_package(Eigen3 QUIET)
find_package(Ceres REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  		${catkin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${CERES_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(path_planning
		src/rrt.cpp
		src/obstacles.cpp)

## Declare a cpp executable
# add_executable(path_planning_node src/path_planning_node.cpp)

add_executable(rrt_node
        src/rrt_node.cpp
        src/path_smoothing.cpp)
add_dependencies(rrt_node
        path_planning)
target_link_libraries(rrt_node
        path_planning
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES})

add_executable(env_node
        src/environment.cpp)
add_dependencies(env_node
        path_planning)
target_link_libraries(env_node
        path_planning
        ${catkin_LIBRARIES} )

