cmake_minimum_required(VERSION 3.0.2)
project(scripts)

# Runnables to current directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/exe)

# Link to the build top-level libraries
add_subdirectory(../.. build)

# Link with ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
)

# Link library fast_turtle to scripts
include_directories(../include ${catkin_INCLUDE_DIRS})

#[[ 
BELOW YOU EDIT ACCORDING TO YOUR EXECUTABLE NAME

example:
If your script is called "main.cpp" you add the following lines:
1) add_executable(any_name_for_your_executable_file main.cpp)
2) target_link_libraries(any_name_for_your_executable_file ft)
]]
add_executable(example example.cpp)
target_link_libraries(example ft ${catkin_LIBRARIES})
add_dependencies(example fast_turtle_generate_messages_cpp)