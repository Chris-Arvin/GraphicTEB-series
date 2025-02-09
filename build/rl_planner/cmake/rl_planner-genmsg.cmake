# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rl_planner: 0 messages, 1 services")

set(MSG_I_FLAGS "-Ivisualization_msgs:/opt/ros/noetic/share/visualization_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rl_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" NAME_WE)
add_custom_target(_rl_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rl_planner" "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" "std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:visualization_msgs/Marker:std_msgs/ColorRGBA:geometry_msgs/Vector3:geometry_msgs/Point:visualization_msgs/MarkerArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rl_planner
  "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rl_planner
)

### Generating Module File
_generate_module_cpp(rl_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rl_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rl_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rl_planner_generate_messages rl_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" NAME_WE)
add_dependencies(rl_planner_generate_messages_cpp _rl_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rl_planner_gencpp)
add_dependencies(rl_planner_gencpp rl_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rl_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rl_planner
  "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rl_planner
)

### Generating Module File
_generate_module_eus(rl_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rl_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rl_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rl_planner_generate_messages rl_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" NAME_WE)
add_dependencies(rl_planner_generate_messages_eus _rl_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rl_planner_geneus)
add_dependencies(rl_planner_geneus rl_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rl_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rl_planner
  "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rl_planner
)

### Generating Module File
_generate_module_lisp(rl_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rl_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rl_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rl_planner_generate_messages rl_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" NAME_WE)
add_dependencies(rl_planner_generate_messages_lisp _rl_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rl_planner_genlisp)
add_dependencies(rl_planner_genlisp rl_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rl_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(rl_planner
  "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rl_planner
)

### Generating Module File
_generate_module_nodejs(rl_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rl_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rl_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rl_planner_generate_messages rl_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" NAME_WE)
add_dependencies(rl_planner_generate_messages_nodejs _rl_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rl_planner_gennodejs)
add_dependencies(rl_planner_gennodejs rl_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rl_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rl_planner
  "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rl_planner
)

### Generating Module File
_generate_module_py(rl_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rl_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rl_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rl_planner_generate_messages rl_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arvin/Documents/pedsim_ws/src/rl_planner/srv/rl_state.srv" NAME_WE)
add_dependencies(rl_planner_generate_messages_py _rl_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rl_planner_genpy)
add_dependencies(rl_planner_genpy rl_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rl_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rl_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rl_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(rl_planner_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rl_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rl_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rl_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(rl_planner_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rl_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rl_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rl_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(rl_planner_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rl_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rl_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rl_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(rl_planner_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rl_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rl_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rl_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rl_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(rl_planner_generate_messages_py visualization_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rl_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
