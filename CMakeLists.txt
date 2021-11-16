cmake_minimum_required(VERSION 3.0.2)
project(fast_turtle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# If custom messages uncomment this
add_message_files(
   FILES
   RobotData.msg
   RobotDataArray.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fast_turtle
  #CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fast_turtle.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fast_turtle_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fast_turtle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Include headers
include_directories(include)

# Libraries definition
add_library(ft STATIC
  src/fast_turtle.cpp
  src/goal.cpp
  src/geometry.cpp
  src/utils.cpp
  src/obstacles.cpp
  src/robot.cpp
  src/world.cpp
  src/swarm_robotics.cpp
  src/controller.cpp
)

# Link libraries
target_link_libraries(ft ${catkin_LIBRARIES})

# It defines the live_simulation.cpp as an executable
add_executable(live_simulation src/live_simulation.cpp)
target_link_libraries(live_simulation ${catkin_LIBRARIES} ft)
add_dependencies(live_simulation fast_turtle_generate_messages_cpp)

# It defines the arena01.cpp as an executable
add_executable(arena01 src/arena01.cpp)
target_link_libraries(arena01 ${catkin_LIBRARIES} ft)
add_dependencies(arena01 fast_turtle_generate_messages_cpp)

# It defines the arena00.cpp as an executable
add_executable(arena00 src/arena00.cpp)
target_link_libraries(arena00 ${catkin_LIBRARIES} ft)
add_dependencies(arena00 fast_turtle_generate_messages_cpp)

# It defines the test_drones.cpp as an executable
add_executable(test_drones src/test_drones.cpp)
target_link_libraries(test_drones ${catkin_LIBRARIES} ft)
add_dependencies(test_drones fast_turtle_generate_messages_cpp)

# It defines the my_controller.cpp as an executable
add_executable(my_controller src/my_controller.cpp)
target_link_libraries(my_controller ${catkin_LIBRARIES} ft)
add_dependencies(my_controller fast_turtle_generate_messages_cpp)

# It defines the my_controller2.cpp as an executable
add_executable(my_controller2 src/my_controller2.cpp)
target_link_libraries(my_controller2 ${catkin_LIBRARIES} ft)
add_dependencies(my_controller2 fast_turtle_generate_messages_cpp)

# It defines the arena_simple.cpp as an executable
add_executable(arena_simple src/arena_simple.cpp)
target_link_libraries(arena_simple ${catkin_LIBRARIES} ft)
add_dependencies(arena_simple fast_turtle_generate_messages_cpp)

# It defines the arena_obstacle.cpp as an executable
add_executable(arena_obstacle src/arena_obstacle.cpp)
target_link_libraries(arena_obstacle ${catkin_LIBRARIES} ft)
add_dependencies(arena_obstacle fast_turtle_generate_messages_cpp)

# It defines the arena_one_obstacle.cpp as an executable
add_executable(arena_one_obstacle src/arena_one_obstacle.cpp)
target_link_libraries(arena_one_obstacle ${catkin_LIBRARIES} ft)
add_dependencies(arena_one_obstacle fast_turtle_generate_messages_cpp)

# It defines the arena_one_obstacle.cpp as an executable
add_executable(arena_one_obstacle_two src/arena_one_obstacle_two.cpp)
target_link_libraries(arena_one_obstacle_two ${catkin_LIBRARIES} ft)
add_dependencies(arena_one_obstacle_two fast_turtle_generate_messages_cpp)

