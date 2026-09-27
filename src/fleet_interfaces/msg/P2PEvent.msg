# Connection / Disconnection / Dropout event notification
string robot_id
string peer_id
uint8 event_type
float64 distance
builtin_interfaces/Time timestamp

uint8 EVENT_CONNECTED = 1
uint8 EVENT_DISCONNECTED = 2
uint8 EVENT_DROPOUT = 3
