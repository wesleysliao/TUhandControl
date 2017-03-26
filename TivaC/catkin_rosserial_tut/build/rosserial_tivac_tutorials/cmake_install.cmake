# Install script for directory: /home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac_tutorials/msg" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/msg/Buttons.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac_tutorials/srv" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/srv/ColorRGBA.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac_tutorials/cmake" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/build/rosserial_tivac_tutorials/catkin_generated/installspace/rosserial_tivac_tutorials-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/devel/include/rosserial_tivac_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/devel/share/roseus/ros/rosserial_tivac_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/devel/share/common-lisp/ros/rosserial_tivac_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/devel/share/gennodejs/ros/rosserial_tivac_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/devel/lib/python2.7/dist-packages/rosserial_tivac_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/devel/lib/python2.7/dist-packages/rosserial_tivac_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/build/rosserial_tivac_tutorials/catkin_generated/installspace/rosserial_tivac_tutorials.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac_tutorials/cmake" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/build/rosserial_tivac_tutorials/catkin_generated/installspace/rosserial_tivac_tutorials-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac_tutorials/cmake" TYPE FILE FILES
    "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/build/rosserial_tivac_tutorials/catkin_generated/installspace/rosserial_tivac_tutorialsConfig.cmake"
    "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/build/rosserial_tivac_tutorials/catkin_generated/installspace/rosserial_tivac_tutorialsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac_tutorials" TYPE FILE FILES "/home/wesley/Sources/TUhandControl/TivaC/catkin_rosserial_tut/src/rosserial_tivac_tutorials/package.xml")
endif()

