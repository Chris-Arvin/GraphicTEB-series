### catkin_make
```bash
source /opt/ros/noetic/setup.bash
cd Documents/pedsim_ws
catkin_make
# option? catkin_make -DCMAKE_CXX_STANDARD=17
```

### run the code
1. pedsim_simulator
```bash
source /opt/ros/noetic/setup.bash
source /Documents/pedsim_ws/devel/setup.bash
roslaunch pedsim_simulator pedsim_simulator.launch
```
2. move_base
```bash
source /opt/ros/noetic/setup.bash
source /Documents/pedsim_ws/devel/setup.bash
roslaunch move_base move_base.launch
```
3. rl_planner
```bash
source /opt/ros/noetic/setup.bash
source /Documents/pedsim_ws/devel/setup.bash
cd ~/Documents/pedsim_ws/src/rl_planner/TRAIN_SAC && python3 train.py
```


### link projection: 
```bash
sudo ln -s ~/opencv_build/opencv/include /usr/local/include/opencv
sudo ln -s /usr/lib/x86_64-linux-gnu/liborocos-kdl.so /usr/local/lib/liborocos-kdl.so
sudo ln -s /usr/lib/x86_64-linux-gnu/liborocos-kdl.so.1.5.1 /usr/local/lib/liborocos-kdl.so.1.5.1
sudo ln -s /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.12.0 /usr/local/lib/libopencv_core.so.3.4.15
```


### locations
```bash
location of g2o: ~/g2o
location of opencv: ~/opencv_build/opencv
```