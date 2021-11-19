#cmake_minimum_required(VERSION 3.5)
project(lss_ros2_control)

# Default to C99
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rcutils REQUIRED)
#find_package(rmw REQUIRED)
find_package(diagnostic_msgs REQUIRED)
#find_package(humanoid_model_msgs REQUIRED)

include_directories(include)

#if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #  ament_lint_auto_find_test_dependencies()
  #endif()

add_library(ros2_lss_bus SHARED
        include/ros2_lss_bus/ros2_lss_bus.hpp
        src/ros2_lss_bus.cpp
      )

target_link_libraries(ros2_lss_bus lss-bus-static)
add_dependencies(ros2_lss_bus lss-bus-static)
target_include_directories(ros2_lss_bus PRIVATE ${LSS_BUS_INCLUDE_DIR})

if(HAS_LIBFTDI)
  target_link_libraries(ros2_lss_bus ftdi)
endif()
ament_target_dependencies(ros2_lss_bus
        #rcutils
        rclcpp
        hardware_interface
        pluginlib
        diagnostic_msgs
	#        humanoid_model_msgs
      )

#add_executable(lss_joint_states src/lss_joint_states.cpp)
#if(HAS_LIBFTDI)
#    target_link_libraries(lss_joint_states ftdi)
#endif()
#target_link_libraries(lss_joint_states ${LynxmotionLSS_STATIC_LIBRARY})
#ament_target_dependencies(lss_joint_states
#  "rclcpp"
#  "sensor_msgs")

## EXPORTS
ament_export_targets(ros2_lss_bus_exports HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(hardware_interface lss_ros2_control.xml)

ament_export_include_directories(
    include
)
ament_export_libraries(
    ros2_lss_bus lss-bus-static lynxmotion-lss
)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    #    humanoid_model_msgs
    diagnostic_msgs
    humanoid_model_msgs
)

ament_package()

install(
    TARGETS ros2_lss_bus lss-bus-static
    EXPORT ros2_lss_bus_exports
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(
    DIRECTORY include/
    DESTINATION include
)
install(
    DIRECTORY config urdf launch
    DESTINATION share/${PROJECT_NAME}
)


