cmake_minimum_required(VERSION 3.8)
project(aetos_auxiliary)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_pub src/test_publisher.cpp)

ament_target_dependencies(test_pub 
                          rclcpp 
                          std_msgs)

install(TARGETS
          test_pub
DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/receiver.py
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
