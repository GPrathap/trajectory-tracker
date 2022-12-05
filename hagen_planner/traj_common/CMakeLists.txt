cmake_minimum_required(VERSION 3.2)
project(traj_common)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  spline_opt
  xmlrpcpp
  jsk_recognition_msgs
  geometry_msgs
  decom_ros_msgs
  decom_rviz_plugins
  hagen_msgs
  nav_msgs
  motion_model
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_common  
 CATKIN_DEPENDS  spline_opt xmlrpcpp jsk_recognition_msgs geometry_msgs decom_ros_msgs hagen_msgs nav_msgs decom_rviz_plugins motion_model
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_common
    src/planning_visualization.cpp src/planning_saving.cpp src/param_passer.cpp src/math_utils.cpp
    src/corridor.cpp src/display_msg.cpp src/bspline_utils.cpp
    )
target_link_libraries( traj_common
    ${catkin_LIBRARIES} cnpy z casadi
    )  
