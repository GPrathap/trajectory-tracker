cmake_minimum_required(VERSION 3.2)
project(drone_traj_contol)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  nav_msgs
  roscpp
  rospy
  std_msgs
  traj_common
  tf
  visualization_msgs
  mavros_msgs
  hagen_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(EXTERNAL_LIBS cnpy z casadi)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(yaml-cpp)
generate_dynamic_reconfigure_options(cfg/DroneReg.cfg)

catkin_package(
 LIBRARIES drone_traj_contol
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs interactive_markers nav_msgs roscpp rospy std_msgs hagen_msgs tf visualization_msgs traj_common
)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(EXTERNAL_LIBS ${EXTERNAL_LIBS}  yaml-cpp ${catkin_LIBRARIES})

add_executable(drone_path_traj_reg_node src/path_traj_reg.cpp src/path_traj_reg.h)
add_dependencies(drone_path_traj_reg_node ${PROJECT_NAME}_gencfg  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(drone_path_traj_reg_node -Wl,--start-group  ${EXTERNAL_LIBS} -Wl,--end-group)

add_executable(drone_path_traj_contol_take_off src/mav_ros_take_off.cc)
add_dependencies(drone_path_traj_contol_take_off ${PROJECT_NAME}_gencfg  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(drone_path_traj_contol_take_off ${catkin_LIBRARIES})