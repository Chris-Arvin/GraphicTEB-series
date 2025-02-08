# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib;actionlib_msgs;costmap_2d;dynamic_reconfigure;geometry_msgs;mbf_abstract_nav;mbf_costmap_core;mbf_msgs;mbf_utility;nav_core;nav_msgs;pluginlib;roscpp;std_msgs;std_srvs;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmbf_costmap_server;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.74.0".split(';') if "-lmbf_costmap_server;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.74.0" != "" else []
PROJECT_NAME = "mbf_costmap_nav"
PROJECT_SPACE_DIR = "/home/arvin/Documents/pedsim_ws/install"
PROJECT_VERSION = "0.4.0"
