cmake_minimum_required(VERSION 3.2)
project(decom_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
  nav_msgs
  geometry_msgs
  message_generation
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )


add_message_files(
  FILES
  Ellipsoid.msg 
  EllipsoidArray.msg
  Polyhedron.msg 
  PolyhedronArray.msg 
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(DEPENDS message_runtime geometry_msgs nav_msgs
              #  INCLUDE_DIRS include
)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
