cmake_minimum_required(VERSION 3.8)
project(blacknav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(motor_controller src/motor_controller.cpp)
add_executable(odom_pub src/odom_pub.cpp)

target_link_libraries(motor_controller pigpiod_if2)
ament_target_dependencies(motor_controller rclcpp)
ament_target_dependencies(motor_controller geometry_msgs)

target_link_libraries(odom_pub pigpiod_if2)
target_link_libraries(odom_pub Eigen3::Eigen)
ament_target_dependencies(odom_pub rclcpp)
ament_target_dependencies(odom_pub nav_msgs)
ament_target_dependencies(odom_pub geometry_msgs)
ament_target_dependencies(odom_pub tf2)
ament_target_dependencies(odom_pub tf2_ros)

install(TARGETS
motor_controller
odom_pub
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY assets/
  DESTINATION share/blacknav/assets/
)

install(
  DIRECTORY urdf/
  DESTINATION share/blacknav/urdf/
)

ament_package()
