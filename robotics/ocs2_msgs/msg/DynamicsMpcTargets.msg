# Dynamics MPC target (reference) trajectories

std_msgs/Header header

####################################################################################################
# Command type selects how state target trajectory samples are parsed:
#   joint_position:  q ref only
#   joint_velocity:  v ref only
#   joint:           both q/v as ref
#   ee_motion_pose:  pose ref only [px, py, pz, qx, qy, qz, qw]
#   ee_motion_twist: twist ref only [vx, vy, vz, wx, wy, wz]
#   ee_motion:       both pose/twist as ref [px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
####################################################################################################
string command_type

####################################################################################################
# Joint order used by joint state, acceleration, and torque target samples.
# EE motion targets do not need joint_names unless joint acceleration or torque
# target samples are also provided.
####################################################################################################
string[] joint_names

####################################################################################################
# Optional per-joint position/velocity tracking weights.
# Empty means controller default weights. Size 1 means uniform weight. Size N follows joint_names.
####################################################################################################
std_msgs/Float64MultiArray joint_position_tracking_weights
std_msgs/Float64MultiArray joint_velocity_tracking_weights

####################################################################################################
# Optional EE pose/twist tracking weights.
# Empty means controller default weights.
# Size 1 means uniform weight.
# Size 6 for pose weights is [translation xyz, quaternion-vector orientation residual].
# Size 6 for twist weights is [linear velocity, angular velocity].
####################################################################################################
std_msgs/Float64MultiArray ee_motion_pose_tracking_weights
std_msgs/Float64MultiArray ee_motion_twist_tracking_weights

####################################################################################################
# Time trajectory and state target trajectory
####################################################################################################
float64[] time_trajectory
MpcState[] state_trajectory

####################################################################################################
# Input target trajectory
# Input dimension depends on the dynamics model used:
#   inverse dynamics: u = [joint_acceleration, joint_torque, ee_wrench]
#   forward dynamics: u = [joint_torque]
# Optional per-sample joint acceleration target. Empty means zero acceleration.
# Optional per-sample joint torque target. Empty means controller default torque reference.
# Optional per-sample end-effector wrench target [Fx, Fy, Fz, Mx, My, Mz]. Empty means zero wrench.
####################################################################################################
MpcInput[] joint_acceleration_trajectory
MpcInput[] joint_torque_trajectory
MpcInput[] ee_wrench_trajectory
