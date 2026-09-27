# State of an individual robot in the fleet
string robot_id
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist current_velocity
float32 battery_level
uint8 status
string current_task_id

# Status constants
uint8 STATUS_IDLE = 0
uint8 STATUS_NAVIGATING = 1
uint8 STATUS_PICKING_UP = 2
uint8 STATUS_DROPPING_OFF = 3
uint8 STATUS_CHARGING = 4
uint8 STATUS_ERROR = 5
