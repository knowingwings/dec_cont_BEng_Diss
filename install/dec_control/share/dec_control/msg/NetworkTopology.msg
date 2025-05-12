# msg/NetworkTopology.msg
int32 num_robots
float32[] adjacency_matrix
float32[] link_quality       # Quality metric for each link (0-1)
float32[] link_latency      # Communication latency for each link in seconds
float32[] link_reliability  # Reliability metric for each link (0-1)
int64 timestamp             # Timestamp in milliseconds