cmake_minimum_required(VERSION 3.8)
project(my_sample_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(move_moveit src/move_moveit.cpp)
add_executable(haus_des_nikolaus src/move_moveit_anfang.cpp)
add_executable(move_moveit_anfang src/move_moveit_anfang.cpp)
add_executable(moveit_paint_svg src/moveit_paint_svg.cpp)
add_executable(eingener_frame src/eingener_frame.cpp)
add_executable(eingener_frame_svg src/eingener_frame_svg.cpp) 

target_include_directories(move_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  haus_des_nikolaus
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  move_moveit_anfang
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  moveit_paint_svg
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  eingener_frame
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  eingener_frame_svg
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS move_moveit
  move_moveit
  haus_des_nikolaus
  move_moveit_anfang
  moveit_paint_svg
  eingener_frame
  eingener_frame_svg
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
