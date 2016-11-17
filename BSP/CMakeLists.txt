# Set a variable for commands below
get_filename_component(PROJECT_NAME . NAME)

# Define your project and language
project(${PROJECT_NAME} C CXX)

# Define target board
set(${PROJECT_NAME}_BOARD uno)

# Define the source code
file(GLOB ${PROJECT_NAME}_SRCS "*.cpp" "*.h")

# Command to generate code arduino firmware (.hex file)
generate_arduino_library(${PROJECT_NAME})
