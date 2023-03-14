cmake_minimum_required(VERSION 2.8.3)
project(fulanghua_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  orne_waypoints_msgs
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    _Pose.srv
    actions.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    orne_waypoints_msgs
)

catkin_package(
  # LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

