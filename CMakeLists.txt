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

# Set project version number
set(${PROJECT_NAME}_VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set(${PROJECT_NAME}_VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set(${PROJECT_NAME}_VERSION_PATCH 0 CACHE STRING "Project patch version number.")

# Add Arduino libraries to project
link_directories(${ARDUINO_SDK}/libraries)

# Add the project examples directory into build
add_subdirectory(examples)

#====================================================================#
#  Setup Documentation generation                                    #
#====================================================================#

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile})

    add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)

    #install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
endif()