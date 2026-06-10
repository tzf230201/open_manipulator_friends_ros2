cmake_minimum_required(VERSION 3.8)
project(open_manipulator_6dof_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  open_manipulator_msgs
)

add_executable(open_manipulator_6dof_teleop_keyboard
  src/open_manipulator_6dof_teleop_keyboard.cpp
)
target_compile_features(open_manipulator_6dof_teleop_keyboard PUBLIC cxx_std_17)
target_include_directories(open_manipulator_6dof_teleop_keyboard PRIVATE include)
ament_target_dependencies(open_manipulator_6dof_teleop_keyboard ${DEPENDENCIES})

add_executable(open_manipulator_6dof_teleop_joystick
  src/open_manipulator_6dof_teleop_joystick.cpp
)
target_compile_features(open_manipulator_6dof_teleop_joystick PUBLIC cxx_std_17)
target_include_directories(open_manipulator_6dof_teleop_joystick PRIVATE include)
ament_target_dependencies(open_manipulator_6dof_teleop_joystick ${DEPENDENCIES})

install(
  TARGETS
    open_manipulator_6dof_teleop_keyboard
    open_manipulator_6dof_teleop_joystick
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
