cmake_minimum_required(VERSION 2.8.3)
project(TUhand)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DLED_HEARTBEAT)
add_definitions(-DLED_COMM)
add_definitions(-DROSSERIAL_BAUDRATE=1600000)
add_definitions(-DTX_BUFFER_SIZE=512)
add_definitions(-DRX_BUFFER_SIZE=512)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS TUhand.cpp
  BOARD tm4c123gxl
)