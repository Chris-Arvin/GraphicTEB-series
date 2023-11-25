# GraphicTEB
This is the source code of the TASE/ICRA2024 paper [**Improve Computing Efficiency and Motion Safety by Analyzing Environment With Graphics**](https://ieeexplore.ieee.org/document/10210322), an approach to find all the non-homology class trajectories quickly.

[![](https://res.cloudinary.com/marcomontalbano/image/upload/v1668649862/video_to_markdown/images/youtube--SzZGKdbzH9Q-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=SzZGKdbzH9Q "")


## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)
* [Introduction for Parameters](#5-acknowledge)
* [Contributors](#4-Contributors)
* [Acknowledgement](#5-acknowledge)


## 1. Installation
The project has been tested on Ubuntu 18.04 (ROS Melodic). To install the repository, please install some dependence firstly: 
```
$ sudo apt install ros-melodic-navigation
```
Then install OpenCV according the [Chinese reference](https://blog.csdn.net/KIK9973/article/details/118830187) or [English reference](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)

Then please install this project and build it: 
```
$ mkdir -p GraphicTEB_ws/src
$ cd GraphicTEB_ws/src
$ git clone https://github.com/Chris-Arvin/graphicTEB_Improve-Computing-Efficiency-and-Motion-Safety-by-Analyzing-Environment-With-Graphics.git
$ cd ..
$ rosdep install –from-paths src –ignore-src –rosdistro-melodic -y
$ [set the OpenCV_DIR in src/teb_local_planner/CMakeLists.txt according to the real location of your OpenCV]
$ catkin_make
```


## 2. Quick Start
Please open a terminal to launch the pedestrian simulation: 
```
$ source GraphicTEB_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch
```
Open another terminal to launch the navigation simulation: 
```
$ source GraphicTEB_ws/devel/setup.bash
$ roslaunch move_base navigation.launch
```


## 3. Introduction for Key Parameters in "pedsim_simulator.launch"
@param: person_mode
* 0: drive the pedestrian with data replay
* 1: drive the pedestrian with extended social force model
* 2: drive the pedestrian with manual control

@param: robot_mode
* 0: drive the robot with algorithms (baselines or your own algorithm) in the format of the plugin
* 2: drive the robot with manual control

@param: scene_file
* the localization of a .xml file describing the obstacle distribution, pedestrian distribution. 

@param: pose_initial_x, pose_initial_y, pose_initial_theta
* the initial position and orientation of the robot.


## 4. Contributors
* Qianyi Zhang  arvin.nkzqy@gmail.com
* Shichao Wu
* Yuhang Jia
* Yuang xu
* Jingtai Liu

## 5. Acknowledge
This work is based on several open-source works, thanks for their contribution and inspiration: 
* [navigation](https://github.com/ros-planning/navigation)
* [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
* [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)
* [hdl_people_tracking](https://github.com/koide3/hdl_people_tracking)

