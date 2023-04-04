cmake_minimum_required(VERSION 3.5)
project(ublox_msg_filters_external)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(ublox_msg_filters REQUIRED)
find_package(ublox_msgs REQUIRED)

set(dependencies "rclcpp" "message_filters" "ublox_msg_filters" "ublox_msgs")

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker ${dependencies})
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener ${dependencies})
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
