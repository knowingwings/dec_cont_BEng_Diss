# RobotStatus.msg - Status of a robot
std_msgs/Header header

uint32 robot_id                    # Robot ID
geometry_msgs/Pose pose            # Current pose
float64[] capabilities              # Robot capability vector
float64 current_workload           # Current workload
uint32[] assigned_tasks            # IDs of assigned tasks
bool failed                        # Whether the robot has failed
uint64 heartbeat                   # Heartbeat counter for failure detection