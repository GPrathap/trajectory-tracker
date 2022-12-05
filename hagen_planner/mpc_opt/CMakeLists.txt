cmake_minimum_required(VERSION 3.2)
project(mpc_opt)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  map_building_opt
  traj_common
  spline_opt
  rebound_opt
  xmlrpcpp
  decom_rviz_plugins
  decom_ros_msgs
  estimation_opt
)

include_directories(${BOOST_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_opt
  CATKIN_DEPENDS map_building_opt traj_common spline_opt rebound_opt xmlrpcpp decom_rviz_plugins decom_ros_msgs estimation_opt
)

set(EXTERNAL_LIBS cnpy z casadi ${Boost_LIBRARIES} ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(mpc_opt src/nonlinear_mpc_opt.cpp src/mpc_opt.cpp
  src/trajectory_tracker.cpp
  src/trajectory_regulator.cpp
  src/mpc_solver.cpp 
  src/linearized_mpc_opt.cpp
  src/linearized_mpc_trajectory_tracker.cpp
)

target_link_libraries(mpc_opt -Wl,--start-group ${EXTERNAL_LIBS} -Wl,--end-group)
