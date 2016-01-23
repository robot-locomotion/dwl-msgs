cmake_minimum_required(VERSION 2.8.3)
project(dwl_msgs)


find_package(catkin REQUIRED COMPONENTS
  dwl
  roscpp
  rospy
  geometry_msgs
  message_generation)

find_package(dwl REQUIRED)


add_message_files(
   FILES
    Region.msg
    Contact.msg
    ContactSequence.msg
    ContactRegion.msg
    BaseState.msg
    JointState.msg
    JointCommand.msg
    ContactState.msg
    ReducedState.msg
    ReducedTrajectory.msg
    WholeBodyState.msg
    WholeBodyTrajectory.msg
    WholeBodyController.msg)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs)
   

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp message_runtime geometry_msgs
  DEPENDS system_lib)


include_directories(${catkin_INCLUDE_DIRS})
include_directories(${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${dwl_LIBRARY_DIRS})
