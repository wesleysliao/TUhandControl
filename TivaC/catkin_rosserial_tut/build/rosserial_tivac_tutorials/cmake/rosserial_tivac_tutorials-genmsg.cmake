# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rosserial_tivac_tutorials: 1 messages, 1 services")

set(MSG_I_FLAGS "-Irosserial_tivac_tutorials:/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rosserial_tivac_tutorials_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" NAME_WE)
add_custom_target(_rosserial_tivac_tutorials_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosserial_tivac_tutorials" "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" "std_msgs/ColorRGBA"
)

get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" NAME_WE)
add_custom_target(_rosserial_tivac_tutorials_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosserial_tivac_tutorials" "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" "std_msgs/Bool"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Services
_generate_srv_cpp(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Module File
_generate_module_cpp(rosserial_tivac_tutorials
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_tivac_tutorials
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rosserial_tivac_tutorials_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rosserial_tivac_tutorials_generate_messages rosserial_tivac_tutorials_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_cpp _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_cpp _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_tivac_tutorials_gencpp)
add_dependencies(rosserial_tivac_tutorials_gencpp rosserial_tivac_tutorials_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_tivac_tutorials_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Services
_generate_srv_eus(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Module File
_generate_module_eus(rosserial_tivac_tutorials
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosserial_tivac_tutorials
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rosserial_tivac_tutorials_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rosserial_tivac_tutorials_generate_messages rosserial_tivac_tutorials_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_eus _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_eus _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_tivac_tutorials_geneus)
add_dependencies(rosserial_tivac_tutorials_geneus rosserial_tivac_tutorials_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_tivac_tutorials_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Services
_generate_srv_lisp(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Module File
_generate_module_lisp(rosserial_tivac_tutorials
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_tivac_tutorials
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rosserial_tivac_tutorials_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rosserial_tivac_tutorials_generate_messages rosserial_tivac_tutorials_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_lisp _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_lisp _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_tivac_tutorials_genlisp)
add_dependencies(rosserial_tivac_tutorials_genlisp rosserial_tivac_tutorials_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_tivac_tutorials_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Services
_generate_srv_nodejs(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Module File
_generate_module_nodejs(rosserial_tivac_tutorials
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosserial_tivac_tutorials
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rosserial_tivac_tutorials_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rosserial_tivac_tutorials_generate_messages rosserial_tivac_tutorials_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_nodejs _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_nodejs _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_tivac_tutorials_gennodejs)
add_dependencies(rosserial_tivac_tutorials_gennodejs rosserial_tivac_tutorials_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_tivac_tutorials_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Services
_generate_srv_py(rosserial_tivac_tutorials
  "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_tivac_tutorials
)

### Generating Module File
_generate_module_py(rosserial_tivac_tutorials
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_tivac_tutorials
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rosserial_tivac_tutorials_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rosserial_tivac_tutorials_generate_messages rosserial_tivac_tutorials_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_py _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg" NAME_WE)
add_dependencies(rosserial_tivac_tutorials_generate_messages_py _rosserial_tivac_tutorials_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_tivac_tutorials_genpy)
add_dependencies(rosserial_tivac_tutorials_genpy rosserial_tivac_tutorials_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_tivac_tutorials_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_tivac_tutorials)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_tivac_tutorials
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rosserial_tivac_tutorials_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosserial_tivac_tutorials)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosserial_tivac_tutorials
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rosserial_tivac_tutorials_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_tivac_tutorials)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_tivac_tutorials
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rosserial_tivac_tutorials_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosserial_tivac_tutorials)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosserial_tivac_tutorials
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rosserial_tivac_tutorials_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_tivac_tutorials)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_tivac_tutorials\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_tivac_tutorials
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rosserial_tivac_tutorials_generate_messages_py std_msgs_generate_messages_py)
endif()
