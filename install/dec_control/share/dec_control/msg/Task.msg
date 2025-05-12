# Task representation
int32 id
float32[3] position  # 3D position in world frame
float32[] capabilities_required  # Required capabilities
float32 execution_time  # Estimated completion time
int32[] prerequisites  # IDs of prerequisite tasks
bool requires_collaboration  # Whether this task requires both robots