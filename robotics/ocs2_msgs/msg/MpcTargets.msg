# Header
std_msgs/Header header

# Supported command combinations in mpc_controllers:
# base | joint | frame_relation
# base_joint | base_frame_relation | joint_frame_relation
# base_joint_frame_relation
string command_type

string[] joint_names
std_msgs/Float64MultiArray joint_tracking_weights

string[] source_frames
string[] target_frames
std_msgs/Float64MultiArray[] frame_relation_tracking_weights

# For pure base/joint/base_joint commands this contains one trajectory.
# For frame-relation commands it contains one trajectory per frame pair.
# Combined commands put the base/joint/base_joint trajectory first, followed by
# frame-relation trajectories in the same order as source_frames/target_frames.
MpcTargetTrajectories[] target_trajectories
