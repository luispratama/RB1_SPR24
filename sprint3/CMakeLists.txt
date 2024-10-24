cmake_minimum_required(VERSION 3.8)
project(sprint3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # Add OpenCV find package

# Create the executables for place_cylinder and filter_cylinder
add_executable(place_cylinder src/place_cylinder.cpp)
add_executable(filter_cylinder src/filter_cylinder.cpp)

# Link against dependencies for place_cylinder
ament_target_dependencies(
  place_cylinder
  rclcpp
  gazebo_msgs
  geometry_msgs
)

# Link against dependencies for filter_cylinder
ament_target_dependencies(
  filter_cylinder
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# Link against OpenCV for filter_cylinder
target_link_libraries(filter_cylinder ${OpenCV_LIBS})

# Install the executables
install(TARGETS
  place_cylinder
  filter_cylinder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
