# Install script for directory: /home/anupa/UCD/LeoSim/ns3/contrib/leosim

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
    set(CMAKE_INSTALL_CONFIG_NAME "default")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so"
         RPATH "/usr/local/lib:$ORIGIN/:$ORIGIN/../lib:/usr/local/lib64:$ORIGIN/:$ORIGIN/../lib64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/anupa/UCD/LeoSim/ns3/build/lib/libns3.45-leosim-default.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so"
         OLD_RPATH "/home/anupa/UCD/LeoSim/ns3/build/lib:::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:$ORIGIN/:$ORIGIN/../lib:/usr/local/lib64:$ORIGIN/:$ORIGIN/../lib64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.45-leosim-default.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ns3" TYPE FILE FILES
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-beam-manager.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-beam-layout-engine.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-beam-hopping-manager.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-beam-load-balancer.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-sinr-engine.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-channel.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-loader.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-mobility-model.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-multi-beam-model.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-channel-model.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-isl-routing-model.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-routing-calculator.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-weather-model.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-loader-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-mobility-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-channel-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-routing-calculator-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-beam-manager-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-visualization-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-device-installer.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-traffic-generator-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/model/leosim-operator-model.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-operator-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/contrib/leosim/helper/leosim-weather-helper.h"
    "/home/anupa/UCD/LeoSim/ns3/build/include/ns3/leosim-module.h"
    )
endif()

