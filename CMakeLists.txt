#=============================================================================#
# Author: LNSD <lorenzo.delgado@lnsd.es>                                      #
# Date:   16.11.2016                                                          #
#                                                                             #
# Description: Arduino-CMake NRF24 library                                    #
#=============================================================================#

cmake_minimum_required (VERSION 2.8)

# Warn if building in source root
if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message (WARNING "Building into sources dir can be risky, prefer other directory")
endif ()

# Appends the cmake/modules path to MAKE_MODULE_PATH variable.
set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

#====================================================================#
#  Setup ArduinoToolchain                                            #
#====================================================================#

# This line is needed only if you use boards with selectable cpu (like mega, pro etc)
# and this line should go before call to "cmake/ArduinoToolchain.cmake"
# this is durty hack and should be fixed somewhen, because it should go to
# particular cmake subdirectory
# set (ARDUINO_CPU 16MHzatmega328)

set (CMAKE_TOOLCHAIN_FILE cmake/arduino/cmake/ArduinoToolchain.cmake) # Arduino Toolchain

#====================================================================#
#  Setup Library Project                                             #
#====================================================================#

# Set PROJECT_NAME variable
set (PROJECT_NAME NRF24)

# Define your project and language
project (${PROJECT_NAME} C CXX)

# Set project version number
set (${PROJECT_NAME}_VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set (${PROJECT_NAME}_VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set (${PROJECT_NAME}_VERSION_PATCH 0 CACHE STRING "Project patch version number.")

# Add Arduino libraries to project
link_directories (${ARDUINO_SDK}/libraries)

# Add the project examples directory into build
add_subdirectory (examples)

#====================================================================#
#  Setup Documentation generation                                    #
#====================================================================#

include (DocumentationTarget)
