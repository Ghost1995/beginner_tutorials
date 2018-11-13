cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation roslaunch tf)

## Check launch files
roslaunch_add_file_check(launch)

## Specify locations of service files
add_service_files(FILES changeString.srv setMaxCount.srv)

## Generate services with dependencies
generate_messages(DEPENDENCIES std_msgs)

## The catkin_package macro generates cmake config files for your package
catkin_package(CATKIN_DEPENDS message_runtime)

## Specify locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})

## Specify dependencies to link a library or executable target against
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

