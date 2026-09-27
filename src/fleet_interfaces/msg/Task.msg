# Task definition in decentralized warehouse
string task_id
geometry_msgs/PoseStamped pickup_pose
geometry_msgs/PoseStamped dropoff_pose
int32 priority
builtin_interfaces/Time creation_time
uint32 version
uint8 state
string assigned_robot_id

# State constants
uint8 STATE_AVAILABLE = 0
uint8 STATE_ALLOCATED = 1
uint8 STATE_ASSIGNED = 1
uint8 STATE_IN_PROGRESS = 2
uint8 STATE_PICKUP_COMPLETED = 6
uint8 STATE_DELIVERING = 7
uint8 STATE_COMPLETED = 3
uint8 STATE_FAILED = 4
uint8 STATE_CANCELLED = 5
uint8 STATE_BLOCKED = 8
