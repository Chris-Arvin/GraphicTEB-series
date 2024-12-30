# GA-TEB
This is the source code of the GA-TEB (the fourth version of the GraphicTEB series) [**GA-TEB: Goal-Adaptive Framework for Efficient Navigation Based on Goal Lines**](https://arxiv.org/abs/2409.10009), an approach for robot motion planning.
[![](https://github.com/user-attachments/assets/5943bc15-ec92-4a07-8aaa-a8c866e9eb6e)](https://www.youtube.com/watch?v=1K7Klxig8CU)



## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)
* [Introduction for Parameters](#3-Introduction-for-Key-Parameters)
* [Contributors](#4-Contributors)
* [Previous Versions](#5-Previous-Versions)
* [Acknowledgement](#6-Acknowledge)


## 1. Installation
The project has been tested on Ubuntu 18.04 (ROS Melodic). To install the repository, please install some dependence firstly: 
```
$ sudo apt install ros-melodic-navigation
$ sudo apt install ros-melodic-teb-local-planner
```
Then install OpenCV according the [Chinese reference](https://blog.csdn.net/KIK9973/article/details/118830187) or [English reference](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)

Then please install this project and build it: 
```
$ mkdir -p GATEB_ws/src
$ cd GATEB_ws/src
$ git clone https://github.com/Chris-Arvin/GraphicTEB-series.git
$ cd ..
$ rosdep install –from-paths src –ignore-src –rosdistro-melodic -y
$ [set the OpenCV_DIR in src/teb_local_planner/CMakeLists.txt according to the real location of your OpenCV]
$ catkin_make
```


## 2. Quick Start
See the performance of GA-TEB in preset scenes with numerous obstacles: 
```
$ source GATEB_ws/devel/setup.bash
$ roslaunch move_base demo1_navigation.launch
```
Or try another demo with multiple pedestrians: 
```
$ source GATEB_ws/devel/setup.bash
$ roslaunch move_base demo2_navigation.launch
```


## 3. Introduction for Key Parameters

demo1_pedsim_simulator.launch / demo2_pedsim_simulator.launch: 
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
* Zeqing Zhu
* Zhengxi Hu
* Yinuo Song
* Yifan Yang
* Shichao Wu


## 5. Previous Versions
* The first version, <a href="https://ieeexplore.ieee.org/document/10210322">Graphic-TEB</a>, proposes a framework that groups obstacles with computer graphics.
* The second version, <a href="https://github.com/Chris-Arvin/STC-TEB">STC-TEB</a>, adds incremental optimization to this framework.
* In parallel, another version, <a href="https://ieeexplore.ieee.org/document/10161222">CG3</a>, introduces the human gaze to make the robot keep safe distance with human beings adaptively. 


## 6. Acknowledge
This work is based on several open-source works, thanks for their contribution and inspiration: 
* [navigation](https://github.com/ros-planning/navigation)
* [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
* [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)
* [hdl_people_tracking](https://github.com/koide3/hdl_people_tracking)

