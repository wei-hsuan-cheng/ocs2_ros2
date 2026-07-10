# Minimum definition of a Bezier segment.

# If true, the receiver increments the start and end time with its own clock time.
bool use_relative_time

# If true, control points are interpreted relative to the current world-frame pose.
bool use_relative_position

# Dimension of each control point. Currently dimension = 2 is expected by the WB stack.
uint8 dimension

# Segment start and end times. Interpret according to use_relative_time.
float64 start_time
float64 end_time

# Desired forward linear velocities at the segment start and end.
float64 start_velocity
float64 end_velocity

# Cubic Bezier control points in the world frame, optionally relative to the current pose.
geometry_msgs/Vector3[<=4] control_points
