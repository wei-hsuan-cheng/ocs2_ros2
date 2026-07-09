std_msgs/Header header

####################################################################################################
# Frame relation tracking
# string arrays for frame relation pair (source, target)
# frame relation cost function weights for each frame pair; size of 6 [translation xyz, quaternion-vector orientation residual]
# frame relation trajectories: one nav_msgs/Path per frame pair (same order/size
# as the frame arrays); path.poses[j].header.stamp defines each point's timing
####################################################################################################
string[] source_frames
string[] target_frames
std_msgs/Float64MultiArray[] frame_relation_tracking_weights
nav_msgs/Path[] frame_relation_trajectories