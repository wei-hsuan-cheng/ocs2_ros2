std_msgs/Header header

####################################################################################################
# Joint tracking
# double arrays for Joint tracking weights; size of N (should match the dimension of joints)
# Joint Trajectory Containing joint names, position/velocities/accelerationseffort and time_from_start
####################################################################################################
float64[] joint_weights
trajectory_msgs/JointTrajectory joint_trajectory