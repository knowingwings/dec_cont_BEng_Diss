cmake_minimum_required(VERSION 3.22)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)

# Make OpenMANIPULATOR optional
set(ENABLE_OPEN_MANIPULATOR TRUE)
find_package(open_manipulator_msgs QUIET)
if(NOT open_manipulator_msgs_FOUND)
  message(WARNING "open_manipulator_msgs not found, disabling OpenMANIPULATOR support")
  set(ENABLE_OPEN_MANIPULATOR FALSE)
  add_definitions(-DNO_OPEN_MANIPULATOR)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Add conditional compilation
if(ENABLE_OPEN_MANIPULATOR)
  # Add components that require open_manipulator_msgs
else()
  # Add alternative components or skip certain features
endif()