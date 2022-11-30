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


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   set(ament_cmake_cpplint_FOUND TRUE)
#   set(ament_cmake_cppcheck_FOUND TRUE)
#   set(ament_cmake_flake8_FOUND TRUE)
#   set(ament_cmake_uncrustify_FOUND TRUE)
#   ament_add_gtest(beginner_tutorials_test test/test.cpp)
#   ament_target_dependencies(beginner_tutorials_test rclcpp std_msgs)
#   rosidl_target_interfaces(beginner_tutorials_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
#   ament_lint_auto_find_test_dependencies()
# endif()




if(BUILD_TESTING)
  find_package(rclcpp REQUIRED)
  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # set(service_files
  #   "srv/InputStr.srv"
  # )
  # rosidl_generate_interfaces(${PROJECT_NAME}
  #   ${service_files}
  #   SKIP_INSTALL
  # )


  function(custom_executable target)
    add_executable(${target} ${ARGN})
    target_compile_definitions(${target}
      PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(${target} "${cpp_typesupport_target}")
    ament_target_dependencies(${target}
      "rclcpp")
  endfunction()

  function(custom_gtest_executable target)
    ament_add_gtest_executable(${target} ${ARGN})
    target_compile_definitions(${target}
      PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(${target} "${cpp_typesupport_target}")
    ament_target_dependencies(${target}
      "rclcpp")
  endfunction()

  macro(custom_launch_test_two_executables test_name executable1 executable2)
    cmake_parse_arguments(_ARG "" "ARGS1;ARGS2;RMW1;RMW2" "" ${ARGN})
    set(TEST_NAME "${test_name}")
    set(TEST_EXECUTABLE1 "$<TARGET_FILE:${executable1}>")
    set(TEST_EXECUTABLE1_ARGS "${_ARG_ARGS1}")
    set(TEST_EXECUTABLE1_NAME "${executable1}")
    set(TEST_RMW_IMPLEMENTATION1 "${_ARG_RMW1}")
    set(TEST_EXECUTABLE2 "$<TARGET_FILE:${executable2}>")
    set(TEST_EXECUTABLE2_ARGS "${_ARG_ARGS2}")
    set(TEST_EXECUTABLE2_NAME "${executable2}")
    set(TEST_RMW_IMPLEMENTATION2 "${_ARG_RMW2}")
    configure_file(
      test/test.cpp
      ${test_name}${target_suffix}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET "${test_name}${target_suffix}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ${_ARG_UNPARSED_ARGUMENTS}
    )
    if(TEST ${test_name}${target_suffix})
      set_tests_properties(${test_name}${target_suffix}
        PROPERTIES DEPENDS "${executable1}${target_suffix} ${executable2}${target_suffix}"
      )
    endif()
  endmacro()

  macro(targets)

    # Service tests single implementation
    custom_launch_test_two_executables(test
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation}
      TIMEOUT 60)

  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")

  call_for_each_rmw_implementation(targets)

  # Service tests single implementation
  custom_gtest_executable(ros_test "test/test.cpp")

endif()  # BUILD_TESTING






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
