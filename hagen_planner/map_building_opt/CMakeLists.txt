cmake_minimum_required(VERSION 3.2)
project(map_building_opt)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  octomap_msgs
  octomap_ros
  traj_common
  decom_rviz_plugins
  decom_ros_msgs
)


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_building_opt
 CATKIN_DEPENDS roscpp std_msgs octomap_ros traj_common decom_rviz_plugins
 decom_ros_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})

add_library( map_building_opt
    src/edtoctomap.cpp
    src/edt_environment.cpp
    )
target_link_libraries( map_building_opt
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} octomap dynamicedt3d
    )
