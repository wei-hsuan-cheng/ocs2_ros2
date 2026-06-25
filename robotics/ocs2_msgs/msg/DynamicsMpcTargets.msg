# Dynamics MPC target trajectories.
#
# command_type selects how state_trajectory samples are parsed:
#   joint_position: target is q only
#   joint_velocity: target is v only
#   joint:          target is [q, v]
#
# Input target trajectories are separated by physical meaning. Consumers map
# these fields into their own model input:
#   inverse dynamics: u = [joint_acceleration, joint_torque, ee_wrench]
#   forward dynamics: u = [joint_torque]
std_msgs/Header header

# Supported values: joint_position, joint_velocity, joint
string command_type

# Joint order used by state, acceleration, and torque target samples.
string[] joint_names

# Optional per-joint position tracking weights.
# Empty means controller default weights. Size 1 means uniform weight. Size N follows joint_names.
std_msgs/Float64MultiArray joint_position_tracking_weights

# Optional per-joint velocity tracking weights.
# Empty means controller default weights. Size 1 means uniform weight. Size N follows joint_names.
std_msgs/Float64MultiArray joint_velocity_tracking_weights

float64[] time_trajectory
MpcState[] state_trajectory

# Optional per-sample joint acceleration target. Empty means zero acceleration.
MpcInput[] joint_acceleration_trajectory

# Optional per-sample joint torque target. Empty means controller default torque reference.
MpcInput[] joint_torque_trajectory

# Optional per-sample end-effector wrench target [Fx, Fy, Fz, Mx, My, Mz].
# Empty means zero wrench.
MpcInput[] ee_wrench_trajectory
