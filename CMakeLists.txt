cmake_minimum_required(VERSION 3.0.2)
project(bobi_fishbot_interface)

add_compile_options(-std=c++11)


file(GLOB TRACKING_SOURCES
    src/*.cpp
)
list(FILTER INTERFACE_SOURCES EXCLUDE REGEX "fishbot_interface_node.cpp")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    bobi_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS bobi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

message(STATUS ${INTERFACE_SOURCES})

add_executable(fishbot_interface_node src/fishbot_interface_node.cpp ${INTERFACE_SOURCES})
target_link_libraries(fishbot_interface_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(fishbot_interface_node ${catkin_EXPORTED_TARGETS}  bobi_msgs_generate_messages_cpp)
