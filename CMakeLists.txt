cmake_minimum_required(VERSION 2.8.3)
project(picture_taker_client)
## A component package required when building the Catkin.
## Has dependency on message_generation, std_msgs, roscpp.
## An error occurs during the build if these packages do not exist.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)
## Declaration Message: MsgTutorial.msg
add_message_files(FILES MsgDynamixel.msg)
## an option to configure the dependent message.
## An error occurs duing the build if "std_msgs" is not installed.
generate_messages(DEPENDENCIES std_msgs)
## A Catkin package option that describes the library, the Catkin build dependencies,
## and the system dependent packages.
catkin_package(
LIBRARIES picture_taker_client
CATKIN_DEPENDS std_msgs roscpp
)
## Include directory configuration.
include_directories(${catkin_INCLUDE_DIRS})
## Build option for the "topic_publisher" node.
## Configuration of Executable files, target link libraries, and additional dependencies.
add_executable(motor_controller src/motor_controller.cpp)
add_dependencies(motor_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller ${catkin_LIBRARIES})
## Build option for the "topic_subscriber" node.
##add_executable(topic_subscriber src/topic_subscriber.cpp)
##add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS}
##${catkin_EXPORTED_TARGETS})
##target_link_libraries(topic_subscriber ${catkin_LIBRARIES})
