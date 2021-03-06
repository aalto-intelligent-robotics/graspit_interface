cmake_minimum_required(VERSION 2.8.3)
project(graspit_interface)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             shape_msgs
             geometry_msgs
             sensor_msgs
             roscpp
             actionlib
             actionlib_msgs
             graspit_msgs
             scene_completion_msgs)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)
find_package(
  Qt4
  COMPONENTS QtCore QtGui
  REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(graspit)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# To declare and build messages, services or actions from within this package,
# follow these steps: * Let MSG_DEP_SET be the set of packages whose message
# types you use in your messages/services/actions (e.g. std_msgs,
# actionlib_msgs, ...). * In the file package.xml: * add a build_depend tag for
# "message_generation" * add a build_depend and a run_depend tag for each
# package in MSG_DEP_SET * If MSG_DEP_SET isn't empty the following dependency
# has been pulled in but can be declared for certainty nonetheless: * add a
# run_depend tag for "message_runtime" * In this file (CMakeLists.txt): * add
# "message_generation" and every package in MSG_DEP_SET to find_package(catkin
# REQUIRED COMPONENTS ...) * add "message_runtime" and every package in
# MSG_DEP_SET to catkin_package(CATKIN_DEPENDS ...) * uncomment the add_*_files
# sections below as needed and list every .msg/.srv/.action file to be processed
# * uncomment the generate_messages entry below * add every package in
# MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  Body.msg
  GraspableBody.msg
  Robot.msg
  TactileSensorData.msg
  Grasp.msg
  Planner.msg
  SearchSpace.msg
  SearchContact.msg
  Energy.msg
  Contact.msg
  SimAnnParams.msg)

# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  AddObject.srv
  GetRobot.srv
  GetGraspableBody.srv
  GetBody.srv
  GetBodies.srv
  GetRobots.srv
  GetGraspableBodies.srv
  SetGraspableBodyPose.srv
  SetRobotPose.srv
  SetBodyPose.srv
  GetDynamics.srv
  SetDynamics.srv
  SetRobotDesiredDOF.srv
  ForceRobotDOF.srv
  MoveDOFToContacts.srv
  AutoGrasp.srv
  AutoOpen.srv
  ImportRobot.srv
  ImportObstacle.srv
  ImportGraspableBody.srv
  LoadWorld.srv
  SaveWorld.srv
  ClearWorld.srv
  SaveImage.srv
  ToggleAllCollisions.srv
  ComputeQuality.srv
  ComputeEnergy.srv
  ApproachToContact.srv
  FindInitialContact.srv
  DynamicAutoGraspComplete.srv)

add_action_files(
  DIRECTORY
  action
  FILES
  PlanGrasps.action
  PlanXBestGrasps.action
  ExecuteXBestGrasps.action
  SimulationExperiment.action
  GetSegmentedMeshedScene.action
  CompleteMesh.action)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  shape_msgs)

# ##############################################################################
# Declare ROS dynamic reconfigure parameters ##
# ##############################################################################

# To declare and build dynamic reconfigure parameters within this package,
# follow these steps: * In the file package.xml: * add a build_depend and a
# run_depend tag for "dynamic_reconfigure" * In this file (CMakeLists.txt): *
# add "dynamic_reconfigure" to find_package(catkin REQUIRED COMPONENTS ...) *
# uncomment the "generate_dynamic_reconfigure_options" section below and list
# every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options( cfg/DynReconf1.cfg cfg/DynReconf2.cfg )

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if you
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(
  INCLUDE_DIRS
  include
  # LIBRARIES graspit_interface
  CATKIN_DEPENDS
  geometry_msgs
  std_msgs
  message_runtime
  shape_msgs
  # DEPENDS system_lib
)

# ##############################################################################
# Build ##
# ##############################################################################

include(${QT_USE_FILE})
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${GRASPIT_INCLUDE_DIRS}
                    ${QT_INCLUDES})

add_definitions(${QT_DEFINITIONS})

set(MOCS include/graspit_interface.h)

qt4_wrap_cpp(GENERATED_SOURCES ${MOCS})

# Declare a C++ library add_library(graspit_interface
# src/${PROJECT_NAME}/graspit_interface.cpp )

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure add_dependencies(graspit_interface
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
add_library(graspit_interface src/graspit_interface.cpp src/main.cpp
                              ${GENERATED_SOURCES} ${MOCS})

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(graspit_interface ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS} ${QT_DEFINITIONS})
add_dependencies(graspit_interface ${PROJECT_NAME}_gencpp)

# Specify libraries to link a library or executable target against
target_link_libraries(graspit_interface ${catkin_LIBRARIES} ${QT_LIBRARIES}
                      ${PCL_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

# all install targets should use catkin DESTINATION variables See
# http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation in contrast to
# setup.py, you can choose the destination install(PROGRAMS
# scripts/my_python_script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark executables and/or libraries for installation install(TARGETS
# graspit_interface graspit_interface_node ARCHIVE DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark cpp header files for installation install(DIRECTORY
# include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES # myfile1 # myfile2 DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION} )

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_graspit_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test) target_link_libraries(${PROJECT_NAME}-test
# ${PROJECT_NAME}) endif()

# Add folders to be run by python nosetests catkin_add_nosetests(test)
