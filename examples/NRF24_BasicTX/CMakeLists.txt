# Set a variable for commands below
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

# Define your project and language
project(${PROJECT_NAME} C CXX)

# Define target board
set(${PROJECT_NAME}_BOARD uno)

# Define Arduino sketch
set(${PROJECT_NAME}_SKETCH ${CMAKE_CURRENT_SOURCE_DIR})

# Set NRF24 library dependency
link_directories(${CMAKE_SOURCE_DIR})

# Define the port for uploading code to the Arduino
set(${PROJECT_NAME}_PORT /dev/tty.usbmodem1721)

# Command to generate code arduino firmware (.hex file)
generate_arduino_firmware(${PROJECT_NAME})
