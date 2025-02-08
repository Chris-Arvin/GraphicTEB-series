### catkin_make
```bash
source /opt/ros/noetic/setup.bash
cd Documents/pedsim_ws
catkin_make
# option? catkin_make -DCMAKE_CXX_STANDARD=17
```



### linke projection: 
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