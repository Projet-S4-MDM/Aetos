cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)


  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)


  add_executable(talker src/publisher.cpp)
  ament_target_dependencies(talker rclcpp std_msgs)

  add_executable(listener src/subscriber.cpp)
  ament_target_dependencies(listener rclcpp std_msgs)

  install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})
  

  set(ament_cmake_copyright_FOUND TRUE)
  

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
