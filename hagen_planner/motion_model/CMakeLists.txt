cmake_minimum_required(VERSION 3.2)
project(motion_model)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -g -Wextra -fPIC -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  spline_opt
)

# set(BOOST_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu/")
find_package(Boost  COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
include_directories(${BOOST_INCLUDE_DIRS})

# include_directories(/usr/local/lib/)
# link_directories(/usr/local/lib/)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES motion_model
 CATKIN_DEPENDS spline_opt
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

set(TRACKING_ESTIMATION_LIB ${TRACKING_ESTIMATION_LIB} ${Boost_LIBRARIES})
set(TRACKING_ESTIMATION_LIB  ${TRACKING_ESTIMATION_LIB}   ${catkin_LIBRARIES} cnpy z Eigen3::Eigen OsqpEigen::OsqpEigen)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O2 -Wall")

add_library(motion_model src/model.cpp src/spatial_bicycle_models.cpp src/temporal_state.cpp src/spatial_state.cpp)

target_link_libraries(motion_model -Wl,--start-group  ${TRACKING_ESTIMATION_LIB} -Wl,--end-group)




