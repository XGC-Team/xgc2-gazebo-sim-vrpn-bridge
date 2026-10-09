cmake_minimum_required(VERSION 3.16)
project(gazebo_sim_vrpn_bridge)

find_package(catkin REQUIRED COMPONENTS
  xgc2_gazebo_scene
  geometry_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(gazebo REQUIRED)
# ROS Noetic's VRPNConfig consumes CMake's legacy thread-probe variable while
# resolving the server library. Modern FindThreads exposes Threads_FOUND and
# Threads::Threads instead, so translate that successful dependency probe at
# this third-party package boundary.
find_package(Threads REQUIRED)
set(CMAKE_HAVE_THREADS_LIBRARY "${Threads_FOUND}")
find_package(VRPN REQUIRED)
find_package(xgc2_math REQUIRED CONFIG)
find_package(jsoncpp CONFIG REQUIRED)
if(NOT TARGET jsoncpp_lib)
  message(FATAL_ERROR "Focal jsoncpp must export its shared jsoncpp_lib target")
endif()
set(XGC2_VRPN_JSONCPP_TARGET jsoncpp_lib)
find_package(yaml-cpp REQUIRED)
find_library(VRPN_NATIVE_SERVER_LIBRARY NAMES vrpnserver HINTS ${VRPN_INCLUDE_DIR}/../lib REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_sim_vrpn_server_core gazebo_sim_vrpn_system_plugin
  CATKIN_DEPENDS xgc2_gazebo_scene geometry_msgs roscpp tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${VRPN_INCLUDE_DIRS}
)

add_library(gazebo_sim_vrpn_server_core SHARED
  src/server_config.cpp
  src/native_vrpn_extension.cpp
  src/vrpn_tracker_server.cpp
)
target_compile_features(gazebo_sim_vrpn_server_core PUBLIC cxx_std_20)
add_dependencies(gazebo_sim_vrpn_server_core
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_sim_vrpn_server_core
  PUBLIC
    ${catkin_LIBRARIES}
    ${VRPN_NATIVE_SERVER_LIBRARY} ${VRPN_LIBRARIES}
    ${XGC2_VRPN_JSONCPP_TARGET} yaml-cpp
    xgc2_math::math
    Threads::Threads
)

add_library(gazebo_sim_vrpn_system_plugin SHARED
  src/gazebo_vrpn_system_plugin.cpp
)
target_compile_features(gazebo_sim_vrpn_system_plugin PRIVATE cxx_std_20)
add_dependencies(gazebo_sim_vrpn_system_plugin
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_sim_vrpn_system_plugin
  PRIVATE
    gazebo_sim_vrpn_server_core
    ${GAZEBO_LIBRARIES}
)

install(TARGETS
  gazebo_sim_vrpn_server_core
  gazebo_sim_vrpn_system_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(gazebo_sim_vrpn_bridge_mocap_noise_tests test/mocap_noise_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_mocap_noise_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_mocap_noise_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_mocap_noise_tests ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(gazebo_sim_vrpn_bridge_measurement_delay_tests test/measurement_delay_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_measurement_delay_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_measurement_delay_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_measurement_delay_tests ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(gazebo_sim_vrpn_bridge_wire_timestamp_tests test/wire_timestamp_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_wire_timestamp_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_wire_timestamp_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_wire_timestamp_tests ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(gazebo_sim_vrpn_bridge_model_identity_tests test/model_identity_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_model_identity_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_model_identity_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_model_identity_tests gazebo_sim_vrpn_server_core)
  endif()
  add_executable(server_config_native_test test/server_config_native_test.cpp)
  target_compile_features(server_config_native_test PRIVATE cxx_std_20)
  target_link_libraries(server_config_native_test gazebo_sim_vrpn_server_core)
  add_test(NAME vrpn_native_config COMMAND server_config_native_test ${CMAKE_CURRENT_SOURCE_DIR}/config)
endif()
