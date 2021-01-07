# Combined A* with RRT* in ROS
## Overview

This is a package for a generalized motion planning stack developed for ROS Melodic & Ubuntu 18.04. The package is an implementation of hierarchical motion planning where-in giving complete flexibility to the user to choose & work with motion planning algorithms of your choice. The package currently uses A* as a global planner for optimal planning & RRT* as a local planner for quick replanning on encountering static unforseen obstacles & thus helps to plan in static environments under uncertainty

The package allows user to choose any of discrete motion planning algorithm as (A*, Dijkstra, D* ,etc) and even any of sampling based motion planning algorithm such as (RRTs, PRMs, etc) as local / global planner as per choice.

**Keywords:** hierarchical motion planning, planning under uncertainty, A*, RRT*


### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author: Kavit Shah, Soumya Srilekha Balijepally, Akshata Pore<br />
Affiliation: [Worcester Polytechnic Institute](https://www.anybotics.com/)<br />
Maintainer: Kavit Shah, kshah@wpi.edu**

The package has been tested under [ROS]  Melodic on respectively 18.04

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

<!--
[![Build Passing]()]()
-->

<p float="left">
  <img src="/media/hmp_Astar1_9x.gif" width="400" />
  <img src="/media/hmp_Astar2_9x.gif" width="400" /> 
</p>





## Installation


### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (Melodic Preferred),
	- You will need turtlebot3 packages, to download them, use :
		
		sudo apt-get install ros-melodic-turtlebot3*
	
	- You will also need the dwa-local-planner :
		
		sudo apt-get install ros-melodic-dwa-local-planner	

- [Eigen3] (linear algebra library)
- [OpenCV] (Image Processing Library) OpenCV2 and above
- [CMake] Version : 3.10.2 & above




#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/KavitShah1998/Combined_Astar_with_RRTstar_in_ROS
	cd ../
	catkin_make


<!--
### Unit Tests

Run the unit tests with

	catkin_make run_tests_ros_package_template

### Static code analysis

Run the static code analysis with

	catkin_make roslint_ros_package_template
-->
## Usage

Source your catkin_workspace with 
	
	source ./devel/setup.bash

Launch the world file with 
	
	roslaunch hierarchical_motion_planner turtlebot3_world_hmp-w-RVIZ.launch

Launch the planner with

	roslaunch hierarchical_motion_planner turtlebot3_custom_planner.launch

## Config files

Config file config/hmp_planner_params.yaml

* **hmp_planner_params** Defines global & local planners and their parameters


## Launch files

* **turtlebot3_world_hmp.launch:** Launches ROS with Gazebo simulation on world_hmp


* **turtlebot3_world_hmp-w-RVIZ.launch:** Launches ROS with Gazebo simulation on world_hmp along with RVIZ, move_base & amcl

     Arguments 

     - **`map_file.yaml`** - The map file of world obtained as a result of GMapping

* **turtlebot3_custom_planner.launch:** Launches the hierarchical motion planner

* **turtlebot3_custom_planner.launch:** Loads the ros-params defined in config/hmp_planner_params.yaml





## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen3]: http://eigen.tuxfamily.org
[OpenCV]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
