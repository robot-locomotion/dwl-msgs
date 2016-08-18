cmake_minimum_required(VERSION 2.8.3)
project(dwl_msgs)


find_package(catkin REQUIRED COMPONENTS
  dwl
  roscpp
  rospy
  geometry_msgs
  tf
  tf_conversions
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
  CATKIN_DEPENDS  roscpp message_runtime geometry_msgs
  DEPENDS  system_lib)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${dwl_LIBRARY_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}  src/WholeBodyStateInterface.cpp
                             src/ControllerCommons.cpp
                             src/PlannerCommons.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}
                                       ${dwl_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${dwl_msgs_EXPORTED_TARGETS})
