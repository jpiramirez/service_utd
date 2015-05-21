cmake_minimum_required(VERSION 2.8.3)
project(service_utd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  camera_calibration_parsers
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
add_message_files(
  FILES
  Marker.msg
  Markers.msg
  TfDistortInfo.msg
  ProbMap.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
   nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES service_utd
#  CATKIN_DEPENDS camera_calibration_parsers cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(vistarget
#   src/vistarget.cpp
# )

## Declare a cpp executable
add_executable(vispose src/vispose.cpp src/vistarget.cpp)
add_executable(colorpose src/colorpose.cpp src/colortarget.cpp)
add_executable(targetdetector src/targetdetector.cpp src/detecttarget.cpp)
add_executable(unittest src/unittest.cpp src/targetstateestimator.cpp)
add_executable(drone_odom src/drone_odom.cpp)
add_executable(drone_setpoint src/drone_setpoint.cpp)
add_executable(tf2posestamped src/tf2posestamped.cpp)
add_executable(infocontroller src/infocontroller.cpp src/targetstateestimator.cpp)
add_executable(ros2matlab src/ros2matlab.cpp)
add_executable(xml2octave src/xml2octave.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(service_utd_node service_utd_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vispose
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

target_link_libraries(colorpose
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

target_link_libraries(targetdetector
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
 
target_link_libraries(unittest
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

target_link_libraries(drone_odom
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
 
target_link_libraries(drone_setpoint
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
 
target_link_libraries(infocontroller
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

target_link_libraries(tf2posestamped
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
 
target_link_libraries(ros2matlab
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
 
target_link_libraries(xml2octave
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS service_utd service_utd_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_service_utd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


catkin_package(
  CATKIN_DEPENDS message_runtime
)
