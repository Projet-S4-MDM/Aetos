cmake_minimum_required(VERSION 3.8)
project(aetos_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Example.msg"
  "msg/EffectorPosition.msg"
  "msg/MotorVelocity.msg"
  "msg/EncoderValues.msg"
  "msg/Velocity.msg"
  "msg/VelocityArbitration.msg"
  "msg/EncoderArbitration.msg"

  "srv/VelocityArbitration.srv"
  "srv/EncoderArbitration.srv"
)

ament_package()
