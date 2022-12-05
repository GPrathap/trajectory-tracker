cmake_minimum_required(VERSION 3.2)
project(state_machine)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC -fvisibility=hidden")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand -fvisibility=hidden")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug -fvisibility=hidden")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(Boost  COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
include_directories(${BOOST_INCLUDE_DIRS})
find_package(OsqpEigen REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  map_building_opt
  mpc_opt
  traj_common
  message_generation
  mavros
  pcl_conversions
  pcl_ros
  sensor_msgs
  octomap_msgs
  hagen_msgs
  octomap_ros
  spline_opt
  rebound_opt
  laser_geometry
  path_finding_opt
  decom_rviz_plugins
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  hagen_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES state_machine
 CATKIN_DEPENDS map_building_opt  mpc_opt traj_common message_runtime spline_opt rebound_opt path_finding_opt hagen_msgs decom_rviz_plugins  
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(fsm_trajectory_tracker src/fsm_trajectory_tracker.cpp)
target_link_libraries(fsm_trajectory_tracker ${catkin_LIBRARIES} casadi ${Boost_LIBRARIES})
add_dependencies(fsm_trajectory_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} )

add_executable(fsm_linearied_trajectory_tracker src/fsm_linearied_trajectory_tracker.cpp)
target_link_libraries(fsm_linearied_trajectory_tracker ${catkin_LIBRARIES} casadi ${Boost_LIBRARIES} cnpy OsqpEigen::OsqpEigen)
add_dependencies(fsm_linearied_trajectory_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fsm_trajectory_point_stabilizer src/fsm_trajectory_point_stabilizer.cpp )
target_link_libraries(fsm_trajectory_point_stabilizer ${catkin_LIBRARIES} casadi )
add_dependencies(fsm_trajectory_point_stabilizer ${${PROJECT_NAME}_EXPORTED_TARGETS})



