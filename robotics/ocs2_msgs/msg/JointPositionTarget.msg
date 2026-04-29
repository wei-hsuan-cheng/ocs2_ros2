# Arm joint-position target for MPC joint tracking.
#
# If joint_names is empty, every position_trajectory sample and weights entry is
# interpreted in the MPC arm-joint order. If joint_names is provided, consumers
# may reorder samples into their configured MPC arm-joint order.
string[] joint_names

# Optional per-joint tracking weights. Empty means use the configured
# jointTracking weights.
float64[] weights

float64[] time_trajectory
MpcState[] position_trajectory
