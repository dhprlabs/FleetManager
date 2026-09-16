# Spatial-temporal reservation of a warehouse zone / cell
string reservation_id
string robot_id
string segment_id
string task_id
uint8 state
uint64 lamport_clock
builtin_interfaces/Time request_time
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
builtin_interfaces/Time expiry_time
uint8 priority
geometry_msgs/Point zone_min
geometry_msgs/Point zone_max

# Reservation state constants
uint8 STATE_REQUESTED = 0
uint8 STATE_GRANTED = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_RELEASED = 3
uint8 STATE_WAITING = 4
uint8 STATE_EXPIRED = 5
uint8 STATE_DENIED = 6
