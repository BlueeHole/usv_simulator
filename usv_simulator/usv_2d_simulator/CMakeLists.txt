cmake_minimum_required(VERSION 2.8.3)
project(usv_2d_simulator)

add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  util_msgs
  uav_utils
  # cmake_utils
)

###########
## Build ##
###########

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ode)
##
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
  CATKIN_DEPENDS roscpp util_msgs uav_utils
  DEPENDS Eigen3 system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
add_library(usv_dynamics src/dynamics/USV.cpp)

## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
# add_executable(quadrotor_simulator_so3
#   src/quadrotor_simulator_so3.cpp)

# target_link_libraries(quadrotor_simulator_so3 
#    ${catkin_LIBRARIES}
#    quadrotor_dynamics
# )
add_executable(usv_simulator
	src/usv_simulator_node.cpp)
target_link_libraries(usv_simulator 
   ${catkin_LIBRARIES}
   usv_dynamics
)