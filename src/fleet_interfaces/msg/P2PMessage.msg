# Peer-to-peer wireless packet transmitted between robots
string source_robot_id
string target_robot_id  # "*" for neighbor broadcast, or specific robot_id for unicast
string message_type     # STATE_BEACON, TASK_STATUS, MAXSUM_MESSAGE, INTENT, etc.
string payload          # JSON or serialized payload
geometry_msgs/PoseStamped sender_pose
builtin_interfaces/Time timestamp
uint32 sequence_number
