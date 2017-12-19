The dwl-msgs (ROS messages)
==============================================


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

The Robot Operting System (ROS) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. For more details see: http://wiki.ros.org/

The *Dynamic Whole-body Locomotion* library (DWL) describes a set of core functions targeted to developed, design, and deploy locomotion algorithms, i.e. planning, control, etc. DWL is core library used in many projects of the Dynamic Legged Systems Lab of Istituto Italiano di Tecnologia (for more details about the project see http://www.iit.it/en/advr-labs/dynamic-legged-systems.html). For more details about DWL software infrastructure please visit https://github.com/robot-locomotion/dwl.

The dwl_msgs are a set of message description which are compatible with DWL standards. This allows us to define the standarized messages of DWL in ROS.

| [![](https://i.imgur.com/BT7fRCU.gif)](https://www.youtube.com/watch?v=ENHvCGrnr2g&t=2s) | [![](https://i.imgur.com/4kKhryj.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)
|:-------------------------:|:-------------------------:|
| [![](https://i.imgur.com/yXTtxUK.gif)](https://www.youtube.com/watch?v=ArV2yh7KSfE) | [![](https://i.imgur.com/RKe3sNo.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)
|||

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Carlos Mastalli, carlos.mastalli@laas.fr<br />
With support from the Dynamic Legged Systems lab at Istituto Italiano di Tecnologia<br />**


## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The algorithms are built primarily in C++. The library uses a number of the local dependencies, which some of them are optionals. Additionally there are Python scripts for post-processing whole-body states or trajectories.

The dwl-msgs is a ROS packages with the following required dependencies:
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [DWL](https://github.com/robot-locomotion/dwl)


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

Before building the dwl_msgs you need to install the dependencies of DWL. Additionally you have to build dwl with catkin.

The dwl_msgs is a catkin project which can be built as:

	cd your_ros_ws/
	catkin_make
