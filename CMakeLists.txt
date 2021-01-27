## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 3.5)
project(rviz_gui)

# set (CMAKE_CXX_STANDARD 14)
# if(NOT WIN32)
#  set(CMAKE_CXX_FLAGS “${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -fPIC”)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rviz2
  rviz_common
  rviz_default_plugins
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
#   message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
#   ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
#   set(QT_LIBRARIES Qt5::Widgets)
# endif()


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/myviz.cpp
  src/main.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
  rviz_common::rviz_common
  )

## END_TUTORIAL

## Install
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})