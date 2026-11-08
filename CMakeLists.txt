cmake_minimum_required(VERSION 3.8)
project(my_robot_grasp_attach CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# Command parsing is plain C++ so it can be unit tested without Gazebo.
add_library(grasp_attach_command src/grasp_attach_command.cpp)
target_include_directories(grasp_attach_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(grasp_attach_command PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

add_library(gazebo_grasp_attach SHARED src/grasp_attach.cpp)
target_include_directories(gazebo_grasp_attach PRIVATE
  ${gazebo_dev_INCLUDE_DIRS})
target_link_libraries(gazebo_grasp_attach
  grasp_attach_command
  ${gazebo_dev_LIBRARIES})
ament_target_dependencies(gazebo_grasp_attach
  rclcpp
  std_msgs
  gazebo_ros)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gazebo_grasp_attach
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grasp_attach_command test/test_grasp_attach_command.cpp)
  target_link_libraries(test_grasp_attach_command grasp_attach_command)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_plugin_install test/test_plugin_install.py)
endif()

ament_package()
