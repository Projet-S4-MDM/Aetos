cmake_minimum_required(VERSION 3.8)
project(aetos_auxiliary)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aetos_msgs REQUIRED)

add_executable(test_pub src/test_publisher.cpp)
add_executable(serial_com src/serial_com.cpp)

ament_target_dependencies(test_pub 
                          rclcpp 
                          std_msgs)
                          
ament_target_dependencies(serial_com 
                          rclcpp 
                          std_msgs)

install(TARGETS
          test_pub
          serial_com
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
