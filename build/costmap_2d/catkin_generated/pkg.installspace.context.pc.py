# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/usr/include".split(';') if "${prefix}/include;/usr/include/eigen3;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;geometry_msgs;laser_geometry;map_msgs;message_filters;message_runtime;nav_msgs;pluginlib;roscpp;sensor_msgs;std_msgs;tf2_ros;visualization_msgs;voxel_grid".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcostmap_2d;-llayers;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0".split(';') if "-lcostmap_2d;-llayers;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0" != "" else []
PROJECT_NAME = "costmap_2d"
PROJECT_SPACE_DIR = "/home/arvin/Documents/pedsim_ws/install"
PROJECT_VERSION = "1.16.7"
