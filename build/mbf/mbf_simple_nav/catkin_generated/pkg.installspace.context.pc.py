# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib;actionlib_msgs;dynamic_reconfigure;geometry_msgs;mbf_abstract_nav;mbf_msgs;mbf_abstract_core;nav_msgs;pluginlib;roscpp;std_msgs;std_srvs;tf;tf2;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmbf_simple_server;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.74.0".split(';') if "-lmbf_simple_server;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0;/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.74.0" != "" else []
PROJECT_NAME = "mbf_simple_nav"
PROJECT_SPACE_DIR = "/home/arvin/Documents/pedsim_ws/install"
PROJECT_VERSION = "0.4.0"
