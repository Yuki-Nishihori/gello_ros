cmake_minimum_required(VERSION 3.10)
project(gello_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2_camera REQUIRED)
# find_package(spacenav_node REQUIRED)

include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/run_gello.py
  scripts/run_GUI.py
  scripts/run_agent_node.py
  scripts/train.py
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()