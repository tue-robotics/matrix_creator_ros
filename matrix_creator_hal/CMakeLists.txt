cmake_minimum_required(VERSION 2.8.3)
project(matrix_creator_hal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Matrix creator HAL deps
find_library(MATRIX_HAL_LIBRARIES NAMES matrix_creator_hal)
find_library(FFTW_LIBRARIES NAMES fftw3f)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
set(MATRIX_CREATOR_LIBRARIES ${MATRIX_HAL_LIBRARIES} ${FFTW_LIBRARIES} ${WIRINGPI_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(direction_of_arrival_node src/direction_of_arrival_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(direction_of_arrival_node ${catkin_LIBRARIES} ${MATRIX_CREATOR_LIBRARIES})
