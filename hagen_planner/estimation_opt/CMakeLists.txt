cmake_minimum_required(VERSION 3.2)
project(estimation_opt)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -g -Wextra -fPIC -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Boost  COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
include_directories(${BOOST_INCLUDE_DIRS})


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES estimation_opt
 CATKIN_DEPENDS 
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

set(TRACKING_ESTIMATION_LIB ${TRACKING_ESTIMATION_LIB} ${Boost_LIBRARIES})
set(TRACKING_ESTIMATION_LIB  ${TRACKING_ESTIMATION_LIB}  cnpy z Eigen3::Eigen)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O2 -Wall")

add_library(estimation_opt src/kalman.cpp)

target_link_libraries(estimation_opt -Wl,--start-group  ${TRACKING_ESTIMATION_LIB} -Wl,--end-group)
target_link_libraries( estimation_opt ${catkin_LIBRARIES})




