cmake_minimum_required(VERSION 3.8)
project(decentralized_auction_mm)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Check for open_manipulator_msgs
set(HAVE_OPEN_MANIPULATOR FALSE)
find_package(open_manipulator_msgs QUIET)
if(open_manipulator_msgs_FOUND)
  set(HAVE_OPEN_MANIPULATOR TRUE)
  add_definitions(-DHAVE_OPEN_MANIPULATOR)
else()
  message(STATUS "open_manipulator_msgs not found. Building without manipulator support.")
  add_definitions(-DNO_OPEN_MANIPULATOR)
endif()

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/TaskArray.msg"
  "msg/Bid.msg"
  "msg/BidArray.msg"
  "msg/AuctionStatus.msg"
  "msg/RobotStatus.msg"
  "srv/StartAuction.srv"
  "srv/StopAuction.srv"
  "srv/FailRobot.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add library for auction algorithm components
add_library(auction_components
  src/auction/auction_algorithm.cpp
  src/auction/bid_calculator.cpp
  src/auction/consensus_protocol.cpp
  src/auction/recovery_mechanism.cpp
)

# Add library for robot control components
add_library(robot_components
  src/robot/robot_controller.cpp
  src/robot/task_executor.cpp
  src/robot/mobile_manipulator.cpp
)

# Add library for task management components
add_library(task_components
  src/task/task_manager.cpp
  # Update this line if the filename is different
  src/task/dependency_manager.cpp
)

# Add library for utility components
add_library(util_components
  src/util/communication.cpp
  src/util/math_utils.cpp
  src/util/visualization.cpp
)

# Link dependencies
ament_target_dependencies(auction_components
  rclcpp std_msgs geometry_msgs visualization_msgs
)

if(HAVE_OPEN_MANIPULATOR)
  ament_target_dependencies(robot_components
    rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
    turtlebot3_msgs open_manipulator_msgs
  )
else()
  ament_target_dependencies(robot_components
    rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
    turtlebot3_msgs
  )
endif()

ament_target_dependencies(task_components
  rclcpp std_msgs geometry_msgs
)

ament_target_dependencies(util_components
  rclcpp std_msgs geometry_msgs visualization_msgs
)

# Add executables for ROS nodes
add_executable(auction_node src/nodes/auction_node.cpp)
target_link_libraries(auction_node auction_components util_components)

add_executable(robot_node src/nodes/robot_node.cpp)
target_link_libraries(robot_node robot_components auction_components task_components util_components)

add_executable(task_generator_node src/nodes/task_generator_node.cpp)
target_link_libraries(task_generator_node task_components util_components)

# Link dependencies to executables
ament_target_dependencies(auction_node
  rclcpp std_msgs geometry_msgs visualization_msgs
)

if(HAVE_OPEN_MANIPULATOR)
  ament_target_dependencies(robot_node
    rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
    turtlebot3_msgs open_manipulator_msgs
  )
else()
  ament_target_dependencies(robot_node
    rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
    turtlebot3_msgs
  )
endif()

ament_target_dependencies(task_generator_node
  rclcpp std_msgs geometry_msgs
)

# Install targets
install(TARGETS
  auction_components
  robot_components
  task_components
  util_components
  auction_node
  robot_node
  task_generator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/visualize_results.py
  scripts/generate_tasks.py
  DESTINATION lib/${PROJECT_NAME}
)

# Setup Gazebo environment
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add custom messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(auction_node "${cpp_typesupport_target}")
target_link_libraries(robot_node "${cpp_typesupport_target}")
target_link_libraries(task_generator_node "${cpp_typesupport_target}")

ament_package()