cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(
srv_files    
"srv/InputStr.srv") 

rosidl_generate_interfaces(
${PROJECT_NAME}    
${srv_files}    
DEPENDENCIES std_msgs) 
ament_export_dependencies(rosidl_default_runtime)

add_executable(talker src/pub.cpp)
add_executable(listener src/sub.cpp)
add_executable(serv src/Service.cpp)
ament_target_dependencies(talker 
  rclcpp 
  geometry_msgs
  std_msgs
  rclcpp
  tf2
  tf2_ros)
ament_target_dependencies(listener 
  rclcpp 
  std_msgs)
ament_target_dependencies(serv 
  rclcpp)


rosidl_target_interfaces(talker
${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(serv
${PROJECT_NAME} "rosidl_typesupport_cpp")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_add_gtest(beginner_tutorials_test test/test.cpp)
  ament_target_dependencies(beginner_tutorials_test rclcpp std_msgs)
  rosidl_target_interfaces(beginner_tutorials_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  talker
  listener
  serv
  ${PROJECT_NAME}_test 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
