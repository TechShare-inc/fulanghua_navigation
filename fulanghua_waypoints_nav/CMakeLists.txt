cmake_minimum_required(VERSION 2.8.3)
project(fulanghua_waypoints_nav)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  geometry_msgs
  move_base
  roscpp
  actionlib_msgs
  actionlib
  tf
  std_srvs
  fulanghua_srvs
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(waypoints_saver src/waypoints_saver.cpp)
add_executable(waypoints_nav src/waypoints_nav.cpp)

add_dependencies(waypoints_nav ${fulanghua_srvs_EXPORTED_TARGETS})

target_link_libraries(waypoints_saver
    ${catkin_LIBRARIES}
)

target_link_libraries(waypoints_nav
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)


install(TARGETS waypoints_nav waypoints_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
   launch
   src
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

