# Broadcast intent of a robot's movement or target execution
string robot_id
string task_id
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped[] planned_path
builtin_interfaces/Time expected_arrival_time
