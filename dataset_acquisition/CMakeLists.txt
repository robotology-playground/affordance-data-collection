
# Copyright: (C) 2015-2019 APRIL, European Commission H2020 project H2020-MSCA-ITN-2015
# Copyright: (C) 2019 IIT - Istituto Italiano di Tecnologia, Genova, Italy
# 
# Author: Alexandre Antunes <aleksander88@gmail.com>
# CopyPolicy: Released under the terms of the GNU GPL v2.0

# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
set(PROJECTNAME dataset_acquisition)
project(${PROJECTNAME})

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(OpenCV REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# IDL is not implemented here, but I keep the flags just in case
#include(YarpIDL)
set(IDL_GENERATED_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}") 
yarp_idl_to_dir(${PROJECTNAME}.thrift ${IDL_GENERATED_CODE_DIR})


# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

file(GLOB source src/*.cpp ../common/src/*.cpp)
file(GLOB header include/*.h ../common/include/*.h)

source_group("Source files" FILES ${source})
source_group("Header files" FILES ${header})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/../common/include
                    ${IDL_GENERATED_CODE_DIR}/include
                    ${YARP_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

#file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
#yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
