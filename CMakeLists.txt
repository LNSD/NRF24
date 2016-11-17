#=============================================================================#
# Author: LNSD <lorenzo.delgado@lnsd.es>                                      #
# Date:   16.11.2016                                                          #
#                                                                             #
# Description: Arduino-CMake NRF24 library                                    #
#=============================================================================#

# This line is needed only if you use boards with selectable cpu (like mega, pro etc)
# and this line should go before call to "cmake/ArduinoToolchain.cmake"
# this is durty hack and should be fixed somewhen, because it should go to
# particular cmake subdirectory
# set(ARDUINO_CPU 16MHzatmega328)

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain

cmake_minimum_required(VERSION 2.8)

#====================================================================#
#  Setup Library Project                                             #
#====================================================================#

# Set PROJECT_NAME variable
set(PROJECT_NAME NRF24)

# Define your project and language
project(${PROJECT_NAME} C CXX)

print_board_list()
print_programmer_list()

# Add Arduino libraries to project
link_directories(${ARDUINO_SDK}/libraries)

# Add the project examples directory into build
add_subdirectory(examples)