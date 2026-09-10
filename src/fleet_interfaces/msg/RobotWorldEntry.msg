# Full robot state with Lamport logical clock — used in distributed world view
string robot_id
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist current_velocity
float32 battery_level
uint8 status
string current_task_id
string current_task_state
geometry_msgs/PoseStamped destination
uint64 lamport_clock
builtin_interfaces/Time wall_time
string source_robot_id
