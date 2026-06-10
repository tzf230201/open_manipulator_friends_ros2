cmake_minimum_required(VERSION 3.8)
project(open_manipulator_6dof_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(open_manipulator_6dof_libs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Boost REQUIRED)

add_executable(${PROJECT_NAME}
  src/open_manipulator_6dof_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
  trajectory_msgs
  open_manipulator_msgs
  robotis_manipulator
  open_manipulator_6dof_libs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/create_udev_rules
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES 99-open-manipulator-cdc.rules
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
