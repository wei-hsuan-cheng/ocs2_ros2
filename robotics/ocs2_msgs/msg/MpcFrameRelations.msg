std_msgs/Header header

####################################################################################################
# Frame relation tracking
# string arrays for frame relation pair (source, target)
# frame relation cost function weights for each frame pair; size of 6 [translation xyz, quaternion-vector orientation residual]
# frame relation trajectories for each frame pair
####################################################################################################
string[] source_frames
string[] target_frames
std_msgs/Float64MultiArray[] frame_relation_tracking_weights
geometry_msgs/PoseStamped[] frame_relation_trajectory