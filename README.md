# GraphicTEB
This is the source code of the TASE/ICRA2024 paper [**Improve Computing Efficiency and Motion Safety by Analyzing Environment With Graphics**](https://ieeexplore.ieee.org/document/10210322). 

This paper proposes a Graphic-and Timed-Elastic-Band-based approach (GraphicTEB) with spatial completeness and high computing efficiency. 
The environment is analyzed utilizing computer graphics, where obstacles are extracted as nodes and their relationships are built as edges. 
Three contributions are presented. 1) By assembling directed detours formed by nodes and segmented paths formed by edges, a generalized path consisting of nodes and edges derives various normal paths efficiently. 
2) By multiplying two vectors starting from the obstacle point closest to the waypoint and the boundary point farthest from the waypoint, an novel obstacle gradient is introduced to guide safer optimization. 
3) By assigning edges with asymmetric Gaussian model, a trajectory evaluation strategy is designed to reflect the motion tendency and motion uncertainty of dynamic obstacles. 
Qualitative and quantitative simulations demonstrate that the proposed GraphicTEB achieves spatial completeness, higher scene pass rate, and fastest computing efficiency. 
Experiments are implemented in long corridor and broad room scenarios, where the robot goes through gaps safely, finds trajectories quickly, and passes pedestrians politely. 

[![](https://user-images.githubusercontent.com/36269534/226112301-6a9947d6-79c9-4ed5-a1df-e17ba4848b7b.png)](https://youtu.be/SzZGKdbzH9Q "")


## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)
* [Introduction for Key Parameters and Topics](#3-Introduction-for-Key-Parameters-and-Key-Topics)
* [More Examples](#4-More-Examples)
* [How to DIY](#5-Instructions-to-DIY)
* [Contributors](#6-Contributors)
* [Acknowledgement](#7-Acknowledgement)


## 1. Installation
The project has been tested on Ubuntu 18.04 (ROS Melodic) and 20.04 (ROS Noetic). To install the repository, please install some dependence firstly (ubuntu 18.04 as example): 
```
$ sudo apt install ros-melodic-navigation
```
Then install OpenCV according the [Chinese reference](https://blog.csdn.net/KIK9973/article/details/118830187) or [English reference](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html):

Then please install this project and build it: 
```
$ mkdir -p GraphicTEB_ws/src
$ cd GraphicTEB_ws/src
$ git clone https://github.com/Chris-Arvin/graphicTEB.git
$ cd ..
$ rosdep install –from-paths src –ignore-src –rosdistro-melodic -y
$ [set the OpenCV_DIR in src/teb_local_planner/CMakeLists.txt according to the real location of your OpenCV]
$ catkin_make
```


## 2. Quick Start
Please open a terminal to launch the pedestrian simulation: 
```
$ source CorridorROS_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=1 robot_mode:=0
```
open another terminal to launch the navigation simulation: 
```
$ source CorridorROS_ws/devel/setup.bash
$ roslaunch move_base_bridge move_base_bridge.launch
```
Currently, the pedestrians are drven by VSFM, and the robot is driven TEB. 


## 3. Introduction for Key Parameters and Key Topics
#### 3.1 Parameters in "pedsim_simulator.launch"
@param: robot_model
* water: a two-wheel differential mobile robot
* wheeltec: a car-like robot

@param: person_mode
* 0: drive the pedestrian with data replay
* 1: drive the pedestrian with extended social force model
* 2: drive the pedestrian with manual control

@param: robot_mode
* 0: drive the robot with algorithms (baselines or your own algorithm) in the format of the plugin
* 1: drive the robot with extended social force model
* 2: drive the robot with manual control

@param: scene_file
* the localization of a .xml file descriping the obstalce distribution, pedestrian distribution, and the robot state. 


#### 3.2 Parameters in "move_base.launch"
@param: local_plan
* [TEB](https://github.com/rst-tu-dortmund/teb_local_planner): Timed Elastic Band
* [DWA](https://github.com/amslabtech/dwa_planner): Dynamic Window Approach
* [Bezier](https://github.com/marinaKollmitz/lattice_planner): generate paths with Bezier curve, and trace the path with PID controller
* [TBL](https://github.com/marinaKollmitz/human_aware_navigation): a simplified version of Time Bounded Lattice
* [PGL](https://ieeexplore.ieee.org/document/9981332): an improved version of TBL, considering the interaction between the robot and pedestrians as a participant game mode. 
* [MPC](https://github.com/JunshengFu/Model-Predictive-Control): Model Predictive Control

#### 3.3 Topics about the environment
@topic: /map
* do not project the person into the costmap, only including the static obstacles
@topic: /map_with_people
* project the person into the costmap, regarding the persons as dynamic obstacles
@topic: /persons
* the state of the persons, including their pose and velocity


## 4. More Examples to open "pedsim_simulator.launch"
Change the params "person_mode" or "robot_mode" in [Quick Start](#2-Quick-Start) to use the simulation platform variously:

#### Case1: drive the person with data replay
open two terminals and input the commands separately: 
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=0
$ rosbag play $(file_name.bag) /persons:=/persons_recorded
```
note:
* You can easily create your own recorded data according to [Instructions to DIY](#5-Instructions to DIY)
#### Case2: drive the person with improved Social Force Model
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=1
```
#### Case3: drive the person with keyboard
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2
```
note: 
* comment the third line and uncomment the fourth line in "interface_teleop.launch" to control the pedestrian one by one: move the pedestrian with "t", "f", "g", "h", "b", and change the ID with "j" or "k".
* uncomment the third line and comment the fourth line in "interface_teleop.launch" to control a few pedestrians simutanuously. Due to the small number of keys, up to 3 people can be controlled at the same time. 
* if "enable_gaze_control" in "pedsim_simulator.launch" is true, control the human gaze with "8", "4", "6", "2", and change ID with "5". 

#### Case4: drive the robot with provided algorithms
```
$ roslaunch pedsim_simulator pedsim_simulator.launch robot_mode:=0
```
note: 
* You can change the baseline in **move_base.launch**, please change the parameter "local_plan" according to guidance. 
* Also, you can adapt your own algorithm in the format of the plugin. The details about the plugin can be found at [how to create a plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
#### Case5: drive the robot with VSFM
```
$ roslaunch pedsim_simulator pedsim_simulator.launch robot_mode:=1
```
#### Case6: drive the robot with keyboard
```
$ roslaunch pedsim_simulator pedsim_simulator.launch robot_mode:=2
```
note: 
* move the robot with "w", "a", "s", "d", "x". 


## 5. Instructions to DIY

#### 5.1 DIY for simulation environment
We allow users to build its simulation environment in the format of xxx.xml. A reference xml file can be found at **example_env.xml**. 
Slam for the environment is unnecessary, because we bridge the auto map-sending function. 
In .xml file, four parts mush be specified： 
```
<!--Obstacles-->
<!--Potential targets of pedestrians-->
<!--AgentClusters-->
<!--Robot-->
```
Use your environment by replacing the source of your **scene_file** in **pedsim_simulator.launch**

#### 5.2 Create a new recorded data
Firstly, open the environment and manually control persons according to [Quick Start](#2-Quick-Start).
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=0
```
Next, open another terminal to record the person states:
```
$ rosbag record /persons -o $(scene_file.bag)
```

To drive persons with recorded data, open two terminals separately according to [Case1](#Case1-drive-the-person-with-data-replay): 
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=0
$ rosbag play $(scene_file.bag) /persons:=/persons_recorded
```


## 6. Contributors
* Qianyi Zhang  zhangqianyi@mail.nankai.edu.cn
* Yinuo Song
* Zhuoran Wang
* Jiahang WU
* Jingtai Liu


## 7. Acknowledgement
These packages have been developed from [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) and [move_base](https://github.com/ros-planning/navigation). Thanks again to support our work.

