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
find_package(open_manipulator REQUIRED)

# Find MoveIt packages
find_package(moveit_ros_planning_interface QUIET)
find_package(open_manipulator_x_bringup QUIET)

# Check for OpenMANIPULATOR dependencies
if(moveit_ros_planning_interface_FOUND AND open_manipulator_x_bringup_FOUND)
  message(STATUS "OpenMANIPULATOR and MoveIt packages found, building with manipulator support")
  set(HAVE_OPEN_MANIPULATOR TRUE)
  add_definitions(-DHAVE_OPEN_MANIPULATOR)
else()
  message(STATUS "OpenMANIPULATOR or MoveIt packages not found, building without manipulator support")
  set(HAVE_OPEN_MANIPULATOR FALSE)
  add_definitions(-DNO_OPEN_MANIPULATOR)
endif()

# Generate custom messages - FIRST STEP TO ENSURE AVAILABILITY
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

# Get the message includes directory for generated headers
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Setup Gazebo environment
find_package(gazebo REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# FIRST: Add all libraries
add_library(auction_components
  src/auction/auction_algorithm.cpp
  src/auction/bid_calculator.cpp
  src/auction/consensus_protocol.cpp
  src/auction/recovery_mechanism.cpp
)

add_library(robot_components
  src/robot/robot_controller.cpp
  src/robot/task_executor.cpp
  src/robot/mobile_manipulator.cpp
)

add_library(task_components
  src/task/task_manager.cpp
  src/task/dependency_manager.cpp
  src/task/task_representation.cpp
)

add_library(util_components
  src/util/communication.cpp
  src/util/math_utils.cpp
  src/util/visualization.cpp
)

# SECOND: Add all executables
add_executable(auction_node src/nodes/auction_node.cpp)
add_executable(robot_node src/nodes/robot_node.cpp)
add_executable(task_generator_node src/nodes/task_generator_node.cpp)

# THIRD: Add message dependencies to all components
target_link_libraries(auction_components "${cpp_typesupport_target}")
target_link_libraries(robot_components "${cpp_typesupport_target}")
target_link_libraries(task_components "${cpp_typesupport_target}")
target_link_libraries(util_components "${cpp_typesupport_target}")

# Link executables with their components
target_link_libraries(auction_node 
  auction_components 
  util_components
  "${cpp_typesupport_target}"
)

target_link_libraries(robot_node 
  robot_components 
  auction_components 
  task_components 
  util_components
  "${cpp_typesupport_target}"
)

target_link_libraries(task_generator_node 
  task_components 
  util_components
  "${cpp_typesupport_target}"
)

# Make sure the generated message headers are included for each component
foreach(target auction_components robot_components task_components util_components)
  target_include_directories(${target} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  )
endforeach()

# Include directories for executables to find message headers
foreach(target auction_node robot_node task_generator_node)
  target_include_directories(${target} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  )
endforeach()

# FOURTH: Now add dependencies to each target
ament_target_dependencies(auction_components
  rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros
)

# Conditionally add dependencies based on available packages
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
  rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs
)

# Link dependencies to executables
ament_target_dependencies(auction_node
  rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros
)

if(HAVE_OPEN_MANIPULATOR)
  ament_target_dependencies(robot_node
    rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
    turtlebot3_msgs open_manipulator
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

ament_package()