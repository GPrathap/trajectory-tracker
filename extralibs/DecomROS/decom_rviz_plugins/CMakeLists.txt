cmake_minimum_required(VERSION 3.2)
project(decom_rviz_plugins)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug")
find_package(catkin REQUIRED COMPONENTS rviz roscpp )
find_package(catkin_simple REQUIRED)
find_package(cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# set(BOOST_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu/")
find_package(Boost COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
# set(Boost_USE_STATIC_LIBS OFF)
include_directories(${BOOST_INCLUDE_DIRS})


include_directories(${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES decom_rviz_plugins
)

add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_simple()

set(SOURCE_FILES
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
  src/multi_detector.cpp
  ${MOC_FILES})

# cs_add_library(decom_rviz_plugins ${SOURCE_FILES})
# target_link_libraries(decom_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${Boost_LIBRARIES})

cs_add_library(decom_rviz_plugins ${SOURCE_FILES})
target_link_libraries(decom_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})

cs_install()

