# Full task state with Lamport logical clock — used in distributed world view
string task_id
uint8 state
string assigned_robot_id
uint32 version
uint64 lamport_clock
builtin_interfaces/Time wall_time
string source_robot_id
geometry_msgs/PoseStamped pickup_pose
geometry_msgs/PoseStamped dropoff_pose
int32 priority
