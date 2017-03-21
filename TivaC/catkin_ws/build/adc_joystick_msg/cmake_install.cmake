# Install script for directory: /home/wesley/Sources/TUhandControl/TivaC/catkin_ws/src/adc_joystick_msg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adc_joystick_msg/msg" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/src/adc_joystick_msg/msg/ADC_Joystick.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adc_joystick_msg/cmake" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/build/adc_joystick_msg/catkin_generated/installspace/adc_joystick_msg-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/devel/include/adc_joystick_msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/devel/share/roseus/ros/adc_joystick_msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/devel/share/common-lisp/ros/adc_joystick_msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/devel/share/gennodejs/ros/adc_joystick_msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/devel/lib/python2.7/dist-packages/adc_joystick_msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/devel/lib/python2.7/dist-packages/adc_joystick_msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/build/adc_joystick_msg/catkin_generated/installspace/adc_joystick_msg.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adc_joystick_msg/cmake" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/build/adc_joystick_msg/catkin_generated/installspace/adc_joystick_msg-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adc_joystick_msg/cmake" TYPE FILE FILES
    "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/build/adc_joystick_msg/catkin_generated/installspace/adc_joystick_msgConfig.cmake"
    "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/build/adc_joystick_msg/catkin_generated/installspace/adc_joystick_msgConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adc_joystick_msg" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_ws/src/adc_joystick_msg/package.xml")
endif()

