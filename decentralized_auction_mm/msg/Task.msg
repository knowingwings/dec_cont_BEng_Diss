# Task.msg - Representation of an assembly task
std_msgs/Header header

uint32 id                           # Unique task identifier
string name                         # Task name

# Spatial information
geometry_msgs/Pose position         # Task position in world frame
geometry_msgs/Vector3 dimensions    # Task dimensions (for visualization)

# Task characteristics
float64[] capabilities_required     # Required capabilities vector
float64 execution_time              # Estimated execution time
uint32[] prerequisites              # IDs of prerequisite tasks
bool is_collaborative               # Whether task requires both robots

# Task status
uint8 STATUS_PENDING = 0
uint8 STATUS_ASSIGNED = 1
uint8 STATUS_IN_PROGRESS = 2
uint8 STATUS_COMPLETED = 3
uint8 STATUS_FAILED = 4
uint8 status                        # Current task status

# Assignment information
int32 assigned_robot                # Robot ID, -1 if unassigned
float64 progress                    # Progress from 0.0 to 1.0
float64 current_price               # Current price in auction