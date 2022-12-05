cmake_minimum_required(VERSION 3.2)
project(spline_opt)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES spline_opt 
#  CATKIN_DEPENDS traj_common
)

set(EXTERNAL_LIBS cnpy z casadi ${Boost_LIBRARIES} ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)


add_library(spline_opt src/non_uniform_bspline.cpp src/reference_path.cpp)
target_link_libraries(spline_opt -Wl,--start-group ${EXTERNAL_LIBS} -Wl,--end-group)

