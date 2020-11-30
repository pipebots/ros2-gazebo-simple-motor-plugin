cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_motor_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
# find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(trajectory_msgs REQUIRED)

# Messages.
find_package(rosidl_default_generators REQUIRED)

set(message_files
  "msg/MotorControl.msg"
)

set(service_files
)

set(action_files
)

rosidl_generate_interfaces(${PROJECT_NAME}_messages  # FIXME Not sure the name is right.
  ${message_files}
  ${service_files}
  ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)

# gazebo_ros_motor_plugin shared library

set(library_name
  lib${PROJECT_NAME}
)

add_library(${library_name} SHARED
  src/${PROJECT_NAME}.cpp
)

target_include_directories(${library_name} PUBLIC include)

ament_target_dependencies(${library_name}
  "gazebo_dev"
  "gazebo_ros"
  # "geometry_msgs"
  # "nav_msgs"
  "rclcpp"
  # "tf2"
  # "tf2_geometry_msgs"
  # "tf2_ros"
)

ament_export_libraries(${library_name})

install(DIRECTORY include/
        DESTINATION include
)

install(TARGETS
  ${library_name}  # FIXME Not sure the library should be installed where ever this is.
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Inspired by https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/test/CMakeLists.txt
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}.cpp
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT
      120
  )
  target_link_libraries(test_${PROJECT_NAME}
    gazebo_test_fixture
  )
endif()

ament_package()
