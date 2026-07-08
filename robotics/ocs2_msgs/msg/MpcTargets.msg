# Kinematics MPC target (reference) trajectories

std_msgs/Header header

####################################################################################################
# Command type selects how state target trajectory samples are parsed:
# Supported command combinations (determine the cost function activation):
# base | joint | frame_relation
# base_joint | base_frame_relation | joint_frame_relation
# base_joint_frame_relation
# hold_joint
####################################################################################################
string command_type

####################################################################################################
# Joint tracking
# string array for joint tracking
# joint tracking cost function weights for all joints
####################################################################################################
string[] joint_names
std_msgs/Float64MultiArray joint_tracking_weights

####################################################################################################
# Frame relation tracking
# string arrays for frame relation pair (source, target)
# frame relation cost function weights for each frame pair; size of 6 [translation xyz, quaternion-vector orientation residual]
####################################################################################################
string[] source_frames
string[] target_frames
std_msgs/Float64MultiArray[] frame_relation_tracking_weights

####################################################################################################
# MPC target (reference) trajectories
# For pure base/joint/base_joint commands this contains one trajectory.
# For frame-relation commands it contains one trajectory per frame pair.
# Combined commands put the base/joint/base_joint trajectory first, followed by
# frame-relation trajectories in the same order as source_frames/target_frames.
####################################################################################################
MpcTargetTrajectories[] target_trajectories
