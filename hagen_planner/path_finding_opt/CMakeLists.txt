cmake_minimum_required(VERSION 3.2)
project(path_finding_opt)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -g -Wextra -fPIC -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  map_building_opt
#  cv_bridge
  decom_rviz_plugins
  decom_ros_msgs
)

include_directories(/usr/local/lib/)
link_directories(/usr/local/lib/)


find_package(Boost  COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
include_directories(${BOOST_INCLUDE_DIRS})


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_finding_opt
 CATKIN_DEPENDS map_building_opt  decom_rviz_plugins decom_ros_msgs
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)



set(PLANNING_LIB ${PLANNING_LIB} ${Boost_LIBRARIES})
set(PLANNING_LIB  ${PLANNING_LIB}  cnpy z)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O2 -Wall")



add_library(path_finding_opt src/dyn_a_star.cpp src/jps_planner.cpp src/graph_search.cpp)


target_link_libraries(path_finding_opt -Wl,--start-group  
                                ${PLANNING_LIB} -Wl,--end-group)
target_link_libraries( path_finding_opt ${catkin_LIBRARIES})




