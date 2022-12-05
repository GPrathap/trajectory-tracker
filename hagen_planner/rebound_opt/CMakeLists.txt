cmake_minimum_required(VERSION 3.2)
project(rebound_opt)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug")

add_definitions(-DUSE_MOSEK=0)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# find_package(OsqpEigen REQUIRED)
link_directories(${PROJECT_SOURCE_DIR}/../../mosek/tools/platform/linux64x86/bin)
include_directories( ${PROJECT_SOURCE_DIR}/../../mosek/tools/platform/linux64x86/h)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  map_building_opt
  path_finding_opt
#  cv_bridge
  traj_common
  decom_rviz_plugins
  decom_ros_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rebound_opt
 CATKIN_DEPENDS map_building_opt path_finding_opt traj_common decom_rviz_plugins decom_ros_msgs
#  DEPENDS system_lib
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS} 
)

add_library( rebound_opt 
    src/uniform_bspline.cpp 
    src/rebound_optimizer.cpp 
    src/gradient_descent_optimizer.cpp
    )
    
    # fusion64 mosek64  OsqpEigen::OsqpEigen
target_link_libraries( rebound_opt
    ${catkin_LIBRARIES}   
    )  
