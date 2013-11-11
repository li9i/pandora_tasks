# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "simple_communication: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isimple_communication:/home/alek/catkin_ws/src/simple_communication/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(simple_communication_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(simple_communication
  "/home/alek/catkin_ws/src/simple_communication/msg/Num.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_communication
)

### Generating Services

### Generating Module File
_generate_module_cpp(simple_communication
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_communication
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(simple_communication_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(simple_communication_generate_messages simple_communication_generate_messages_cpp)

# target for backward compatibility
add_custom_target(simple_communication_gencpp)
add_dependencies(simple_communication_gencpp simple_communication_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simple_communication_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(simple_communication
  "/home/alek/catkin_ws/src/simple_communication/msg/Num.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simple_communication
)

### Generating Services

### Generating Module File
_generate_module_lisp(simple_communication
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simple_communication
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(simple_communication_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(simple_communication_generate_messages simple_communication_generate_messages_lisp)

# target for backward compatibility
add_custom_target(simple_communication_genlisp)
add_dependencies(simple_communication_genlisp simple_communication_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simple_communication_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(simple_communication
  "/home/alek/catkin_ws/src/simple_communication/msg/Num.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_communication
)

### Generating Services

### Generating Module File
_generate_module_py(simple_communication
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_communication
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(simple_communication_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(simple_communication_generate_messages simple_communication_generate_messages_py)

# target for backward compatibility
add_custom_target(simple_communication_genpy)
add_dependencies(simple_communication_genpy simple_communication_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simple_communication_generate_messages_py)


debug_message(2 "simple_communication: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_communication)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_communication
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(simple_communication_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simple_communication)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simple_communication
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(simple_communication_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_communication)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_communication\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_communication
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(simple_communication_generate_messages_py std_msgs_generate_messages_py)
