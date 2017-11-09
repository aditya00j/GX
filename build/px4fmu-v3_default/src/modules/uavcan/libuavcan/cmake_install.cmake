# Install script for directory: /home/rablsa/Pictures/Firmware/src/modules/uavcan/libuavcan

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uavcan" TYPE DIRECTORY FILES "/home/rablsa/Pictures/Firmware/src/modules/uavcan/libuavcan/dsdl")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rablsa/Pictures/Firmware/build/px4fmu-v3_default/src/modules/uavcan/libuavcan/libuavcan/cmake_install.cmake")
  include("/home/rablsa/Pictures/Firmware/build/px4fmu-v3_default/src/modules/uavcan/libuavcan/libuavcan_drivers/posix/cmake_install.cmake")
  include("/home/rablsa/Pictures/Firmware/build/px4fmu-v3_default/src/modules/uavcan/libuavcan/libuavcan_drivers/stm32/driver/cmake_install.cmake")

endif()

