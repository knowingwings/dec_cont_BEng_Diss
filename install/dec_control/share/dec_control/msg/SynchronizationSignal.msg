# msg/SynchronizationSignal.msg
int32 task_id
int32 robot_id
int32 sync_point
string status  # READY, ACKNOWLEDGE, COMPLETED
int64 timestamp  # milliseconds since epoch