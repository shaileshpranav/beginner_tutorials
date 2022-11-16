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

set(
srv_files    
"srv/InputStr.srv") 

rosidl_generate_interfaces(
${PROJECT_NAME}    
${srv_files}    
DEPENDENCIES builtin_interfaces) 
ament_export_dependencies(rosidl_default_runtime)

add_executable(talker src/pub.cpp)
add_executable(listener src/sub.cpp)
add_executable(serv src/Service.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(serv rclcpp)
install(TARGETS
  talker
  listener
  serv
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rosidl_target_interfaces(serv
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_package()

