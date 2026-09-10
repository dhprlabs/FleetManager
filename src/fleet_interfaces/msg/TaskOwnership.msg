# Declaration or negotiation of task ownership by a robot
string task_id
string robot_id
uint8 status
builtin_interfaces/Time timestamp

uint8 STATUS_CLAIMED = 0
uint8 STATUS_CONFIRMED = 1
uint8 STATUS_RELEASED = 2
uint8 STATUS_CONFLICT = 3
