cmake_minimum_required(VERSION 2.8.3)
project(dwl_msgs)

message("Dynamic Whole-Body Locomotion (DWL) ROS messages")
message("Copyright (C) 2014-2018 Carlos Mastalli, <carlos.mastalli@laas.fr>")
message("All rights reserved.")
message("Released under the BSD 3-Clause License.")



find_package(catkin REQUIRED COMPONENTS
  dwl
  roscpp
  rospy
  geometry_msgs
  tf
  tf_conversions
  message_generation)

catkin_python_setup()

add_message_files(
   FILES
    Vector2.msg
    Region.msg
    Contact.msg
    ContactSequence.msg
    ContactRegion.msg
    BaseState.msg
    JointState.msg
    JointCommand.msg
    ContactState.msg
    ReducedBodyState.msg
    ReducedBodyTrajectory.msg
    WholeBodyState.msg
    WholeBodyTrajectory.msg
    WholeBodyController.msg)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs)
   

catkin_package(
  INCLUDE_DIRS  include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS  roscpp message_runtime geometry_msgs dwl)


# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${dwl_LIBRARY_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}  src/WholeBodyStateInterface.cpp
                             src/ReducedBodyStateInterface.cpp
                             src/ControllerCommons.cpp
                             src/PlannerCommons.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}
                                       ${dwl_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${dwl_msgs_EXPORTED_TARGETS})
