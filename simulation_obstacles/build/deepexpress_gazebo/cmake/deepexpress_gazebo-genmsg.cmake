# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "deepexpress_gazebo: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ideepexpress_gazebo:/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(deepexpress_gazebo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" NAME_WE)
add_custom_target(_deepexpress_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deepexpress_gazebo" "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" "std_msgs/Float64:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(deepexpress_gazebo
  "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deepexpress_gazebo
)

### Generating Services

### Generating Module File
_generate_module_cpp(deepexpress_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deepexpress_gazebo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(deepexpress_gazebo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(deepexpress_gazebo_generate_messages deepexpress_gazebo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" NAME_WE)
add_dependencies(deepexpress_gazebo_generate_messages_cpp _deepexpress_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deepexpress_gazebo_gencpp)
add_dependencies(deepexpress_gazebo_gencpp deepexpress_gazebo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deepexpress_gazebo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(deepexpress_gazebo
  "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deepexpress_gazebo
)

### Generating Services

### Generating Module File
_generate_module_eus(deepexpress_gazebo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deepexpress_gazebo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(deepexpress_gazebo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(deepexpress_gazebo_generate_messages deepexpress_gazebo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" NAME_WE)
add_dependencies(deepexpress_gazebo_generate_messages_eus _deepexpress_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deepexpress_gazebo_geneus)
add_dependencies(deepexpress_gazebo_geneus deepexpress_gazebo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deepexpress_gazebo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(deepexpress_gazebo
  "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deepexpress_gazebo
)

### Generating Services

### Generating Module File
_generate_module_lisp(deepexpress_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deepexpress_gazebo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(deepexpress_gazebo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(deepexpress_gazebo_generate_messages deepexpress_gazebo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" NAME_WE)
add_dependencies(deepexpress_gazebo_generate_messages_lisp _deepexpress_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deepexpress_gazebo_genlisp)
add_dependencies(deepexpress_gazebo_genlisp deepexpress_gazebo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deepexpress_gazebo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(deepexpress_gazebo
  "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deepexpress_gazebo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(deepexpress_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deepexpress_gazebo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(deepexpress_gazebo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(deepexpress_gazebo_generate_messages deepexpress_gazebo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" NAME_WE)
add_dependencies(deepexpress_gazebo_generate_messages_nodejs _deepexpress_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deepexpress_gazebo_gennodejs)
add_dependencies(deepexpress_gazebo_gennodejs deepexpress_gazebo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deepexpress_gazebo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(deepexpress_gazebo
  "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deepexpress_gazebo
)

### Generating Services

### Generating Module File
_generate_module_py(deepexpress_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deepexpress_gazebo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(deepexpress_gazebo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(deepexpress_gazebo_generate_messages deepexpress_gazebo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nabih/simulation_obstacles/src/deepexpress_gazebo/msg/float64stamped.msg" NAME_WE)
add_dependencies(deepexpress_gazebo_generate_messages_py _deepexpress_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deepexpress_gazebo_genpy)
add_dependencies(deepexpress_gazebo_genpy deepexpress_gazebo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deepexpress_gazebo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deepexpress_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deepexpress_gazebo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(deepexpress_gazebo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deepexpress_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deepexpress_gazebo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(deepexpress_gazebo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deepexpress_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deepexpress_gazebo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(deepexpress_gazebo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deepexpress_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deepexpress_gazebo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(deepexpress_gazebo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deepexpress_gazebo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deepexpress_gazebo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deepexpress_gazebo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(deepexpress_gazebo_generate_messages_py std_msgs_generate_messages_py)
endif()
