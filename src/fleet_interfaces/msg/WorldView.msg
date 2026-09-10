# Complete snapshot of a robot's known world view — used for reconnection state exchange
string robot_id
fleet_interfaces/RobotWorldEntry own_state
fleet_interfaces/RobotWorldEntry[] peer_states
fleet_interfaces/TaskWorldEntry[] task_states
uint64 lamport_clock
builtin_interfaces/Time timestamp
