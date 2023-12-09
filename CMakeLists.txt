cmake_minimum_required(VERSION 3.5)
project(rosbag_record_rviz_plugin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17 -DEIGEN_RUNTIME_NO_MALLOC")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fno-asm")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED roslib)

find_package(catkin REQUIRED COMPONENTS
roscpp
class_loader
pluginlib
rviz
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

set(HEADERS
  include/rosbag_record_rviz_plugin.h
  )
qt5_wrap_ui(UIC_FILES
  src/ui/rosbag_record_rviz_plugin.ui
  )
set(SRC_FILES
  src/rosbag_record_rviz_plugin.cpp
  )

add_library(rosbag_record_rviz_plugin SHARED ${SRC_FILES} ${HEADERS} ${UIC_FILES})
target_link_libraries(rosbag_record_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(
  TARGETS rosbag_record_rviz_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
