cmake_minimum_required(VERSION 2.8.3)
project(picture_taker)
## A component package required when building the Catkin.
## Has dependency on message_generation, std_msgs, roscpp.
## An error occurs during the build if these packages do not exist.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp cv_bridge sensor_msgs image_transport)
find_package(OpenCV REQUIRED)
## Declaration Message: MsgTutorial.msg
add_message_files(FILES MsgDynamixel.msg)
add_service_files(FILES image_cmd.srv)
## an option to configure the dependent message.
## An error occurs duing the build if "std_msgs" is not installed.
generate_messages(DEPENDENCIES std_msgs)
## A Catkin package option that describes the library, the Catkin build dependencies,
## and the system dependent packages.
catkin_package(
LIBRARIES picture_taker
CATKIN_DEPENDS std_msgs roscpp
)
## Include directory configuration.
include_directories(${catkin_INCLUDE_DIRS})
## Build option for the "topic_publisher" node.
## Configuration of Executable files, target link libraries, and additional dependencies.
add_executable(motor_controller src/motor_controller.cpp)
add_dependencies(motor_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller ${catkin_LIBRARIES})

add_executable(cam_publisher src/cam_publisher.cpp)
target_link_libraries(cam_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(cam_publisher picture_taker_generate_messages_cpp)

add_executable(picture_server src/picture_server.cpp)
target_link_libraries(picture_server ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(picture_server picture_taker_generate_messages_cpp)
## Build option for the "topic_subscriber" node.
##add_executable(topic_subscriber src/topic_subscriber.cpp)
##add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS}
##${catkin_EXPORTED_TARGETS})
##target_link_libraries(topic_subscriber ${catkin_LIBRARIES})
