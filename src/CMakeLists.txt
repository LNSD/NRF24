# Set a variable for commands below
set(PROJECT_NAME NRF24lib)

# Define your project and language
project(${PROJECT_NAME} C CXX)

set(${PROJECT_NAME}_BOARD uno)

# Define the source code
set(${PROJECT_NAME}_SRCS NRF24lib.cpp)

# Library not detected automatically
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(NRF24_RECURSE True)
set(SPI_RECURSE True)

# Define the port for uploading code to the Arduino
set(${PROJECT_NAME}_PORT /dev/tty.usbmodem1721)

# Command to generate code arduino firmware (.hex file)
generate_arduino_firmware(${PROJECT_NAME})
