cmake_minimum_required(VERSION 2.8.3)
project(image_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  opencv_apps
  cv_bridge
  image_transport
  message_filters
  tf2_ros
  message_generation
)

add_message_files(
   FILES
   StringPointStamped.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_recognition
#  CATKIN_DEPENDS roscpp std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})

add_executable(marker_setter_laser src/marker_setter_laser.cpp)
target_link_libraries(marker_setter_laser ${catkin_LIBRARIES})

add_executable(marker_setter_map src/marker_setter_map.cpp)
target_link_libraries(marker_setter_map ${catkin_LIBRARIES})


