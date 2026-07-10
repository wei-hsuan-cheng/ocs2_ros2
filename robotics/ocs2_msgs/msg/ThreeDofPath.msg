# Minimum definition of a 3-DoF base path.

# If true, the receiver terminates trajectory tracking.
bool stop_execution

# Seconds to wait before starting trajectory tracking.
float64 time_to_start

# If true, desired times are relative to the receiver clock.
bool use_relative_time

# If true, desired points are interpreted relative to the current world-frame pose.
bool use_relative_position

ThreeDofPointTS[] points_to_reach
