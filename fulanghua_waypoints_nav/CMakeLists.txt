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
  fulanghua_action
  rospy
  std_msgs
  orne_waypoints_msgs
  sound_play
  cv_bridge
  image_transport
  camera_action
  fulanghua_msg
  fulanghua_srvs
  sensor_msgs
  # unitree_legged_msgs
)

find_package(PkgConfig)
find_package(OpenCV 4  REQUIRED)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(waypoints_saver src/waypoints_saver.cpp)
add_executable(waypoints_nav src/waypoints_nav.cpp)
add_executable(adv_action_server src/adv_action_server.cpp)
add_executable(align_test src/align_test.cpp)
add_executable(rotate_action_server src/rotate_action_server.cpp)
add_executable(wait_server src/wait_server.cpp)
# add_executable(go1_cmd_server src/go1_high_command.cpp)
add_dependencies(waypoints_nav ${fulanghua_srvs_EXPORTED_TARGETS})

add_executable(aruco_align_server src/aruco_align_action_server.cpp)
target_compile_options(aruco_align_server PRIVATE ${OpenMP_FLAGS})

add_dependencies(aruco_align_server ${catkin_EXPORTED_TARGETS})
add_dependencies(adv_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(align_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(waypoints_saver
    ${catkin_LIBRARIES}
)



target_link_libraries(waypoints_nav
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)


target_link_libraries(adv_action_server
  ${catkin_LIBRARIES}
)
target_link_libraries(align_test
  ${catkin_LIBRARIES}
)
target_link_libraries(rotate_action_server
  ${catkin_LIBRARIES}
)

target_link_libraries(wait_server
  ${catkin_LIBRARIES}
)

target_link_libraries(aruco_align_server 
${OpenCV_LIBRARIES} ${catkin_LIBRARIES} 
${CMAKE_THREAD_LIBS_INIT} ${OpenMP_LIBS}
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