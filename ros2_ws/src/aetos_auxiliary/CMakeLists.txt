cmake_minimum_required(VERSION 3.8)
project(aetos_auxiliary)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aetos_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

pkg_check_modules(UDEV REQUIRED libudev)

add_executable(serial_com src/serial_com.cpp)
add_executable(arbitration src/arbitration.cpp)
                          
ament_target_dependencies(serial_com 
                          rclcpp 
                          std_msgs
                          aetos_msgs)

ament_target_dependencies(arbitration 
                          rclcpp 
                          aetos_msgs)

target_link_libraries(serial_com
          ${UDEV_LIBRARIES}
          Boost::system
          ${CMAKE_THREAD_LIBS_INIT}
        )

install(TARGETS
          serial_com
          arbitration
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
