std_msgs/Header header

####################################################################################################
# Twist tracking
# string arrays for twist pair (source, target)
# the twist command is the body twist of the target frame (expressed in target-frame axes), cmd_vel style
# twist weights for each frame pair; size of 6 (shares same frame relation pose weights, since twist motion is implemented through integrated pose trajectory in kinematics mpc)
# twist command for each frame pair
####################################################################################################
string[] source_frames
string[] target_frames
std_msgs/Float64MultiArray[] twist_tracking_weights
geometry_msgs/TwistStamped[] twist_array