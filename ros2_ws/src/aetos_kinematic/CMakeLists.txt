cmake_minimum_required(VERSION 3.8)
project(aetos_kinematic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aetos_msgs REQUIRED)

add_executable(velocity_conversion src/velocity_conversion.cpp)
ament_target_dependencies(velocity_conversion rclcpp aetos_msgs)

target_include_directories(velocity_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3)

target_compile_features(velocity_conversion PUBLIC c_std_99 cxx_std_17)

install(TARGETS velocity_conversion
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
