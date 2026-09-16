# Observation recorded when a robot arrives at a pickup station
string robot_id
string task_id
geometry_msgs/PoseStamped observed_pose
bool is_valid
string failure_reason
string confirmed_owner_id  # populated by PickupValidator during reconciliation
builtin_interfaces/Time timestamp
string obs_id              # unique: "{robot_id}:{lamport}" for CRDT idempotency
int32 lamport_clock        # Lamport clock at observation time
bool item_present          # True if item physically confirmed present
