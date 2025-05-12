cmake_minimum_required(VERSION 3.8)
project(decentralized_control)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bid.msg"
  "msg/CapabilityUpdate.msg"
  "msg/CollaborationRequest.msg"
  "msg/CollaborationResponse.msg"
  "msg/Heartbeat.msg"
  "msg/NetworkTopology.msg"
  "msg/RecoveryStatus.msg"
  "msg/RobotState.msg"
  "msg/SynchronizationSignal.msg"
  "msg/Task.msg"
  "msg/TaskAssignment.msg"
  "msg/TaskList.msg"
  "msg/TaskPriority.msg"
  "srv/GetAvailableTasks.srv"
  "srv/InitAuction.srv"
  "srv/SubmitBid.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs action_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/analyze_adaptation.py
  scripts/analyze_results.py
  scripts/assembly_task_visualiser.py
  scripts/experiment_runner.py
  scripts/gazebo_physics_configurator.py
  scripts/plot_results.py
  scripts/robot_model_spawner.py
  scripts/run_experiments.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_tests test)
endif()

ament_package()
