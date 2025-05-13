# AuctionStatus.msg - Status of the auction system
std_msgs/Header header

uint8 STATUS_INACTIVE = 0
uint8 STATUS_RUNNING = 1
uint8 STATUS_RECOVERY = 2
uint8 STATUS_COMPLETED = 3
uint8 status

uint32 iteration                   # Current iteration
uint32 total_tasks                 # Total number of tasks
uint32 assigned_tasks              # Number of assigned tasks
uint32 completed_tasks             # Number of completed tasks
uint32 recovery_iteration          # Iteration count in recovery mode (if in recovery)
uint32 failed_robot_id             # ID of failed robot (if in recovery)
float64 global_makespan            # Current estimated makespan