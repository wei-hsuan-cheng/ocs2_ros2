**Mobile Manipulator MPC Policy**

- Topic: `/mobile_manipulator_mpc_policy`
- Type: `ocs2_msgs/msg/MpcFlattenedController`
- Purpose: Publishes the current MPC policy rollout (predicted state/input trajectories, targets, and auxiliary data) used by the mobile manipulator example.

**Message Layout**

- `controller_type: uint8`
  - 0 CONTROLLER_UNKNOWN, 1 CONTROLLER_FEEDFORWARD, 2 CONTROLLER_LINEAR.

- `init_observation: MpcObservation`
  - `time: float64` Current policy start time.
  - `state: MpcState` Current state vector (`float32[] value`).
  - `input: MpcInput` Current input vector (`float32[] value`).
  - `mode: int8` Current mode (for hybrid systems).

- `plan_target_trajectories: MpcTargetTrajectories`
  - `time_trajectory: float64[]` Target times.
  - `state_trajectory: MpcState[]` Target states (per time).
  - `input_trajectory: MpcInput[]` Target inputs (per time).

- `state_trajectory: MpcState[]`
  - Optimized state trajectory from the planner (same length as `time_trajectory`).

- `input_trajectory: MpcInput[]`
  - Optimized input trajectory from the planner (same length as `time_trajectory`).

- `time_trajectory: float64[]`
  - Times corresponding to the states/inputs above.

- `post_event_indices: uint16[]`
  - Indices in the time/state/input arrays immediately after discrete events.

- `mode_schedule: ModeSchedule`
  - `event_times: float64[]` Event times (size = `mode_sequence.size() - 1`).
  - `mode_sequence: int8[]` Sequence of active modes.

- `data: ControllerData[]`
  - Auxiliary payload produced by the “flatten” method, one vector per time index: `ControllerData { float32[] data }`.
  - For the mobile manipulator example (Ridgeback + UR5): this is the 8‑D input vector per step = `[v_x, w_z, qd_0..qd_5]`.

- `performance_indices: MpcPerformanceIndices`
  - Solver metrics (cost, merit, violations, etc.).

**Mobile Manipulator Dimensions**

- State (WheelBasedMobileManipulator): 9 elements
  - Base pose: `[x, y, yaw]` (3)
  - Arm joints: `[q0..q5]` (6)

- Input: 8 elements
  - Base velocities: `[v_x, w_z]` (2)
  - Arm joint velocities: `[qd_0..qd_5]` (6)

- Target state (plan_target_trajectories.state_trajectory): 7 elements
  - End‑effector pose in world frame: `[x, y, z, qx, qy, qz, qw]`.

**Quick Inspection**

- Echo once: `ros2 topic echo /mobile_manipulator_mpc_policy --once`
- Plot a single horizon element with rqt_plot (use array indices):
  - Target EE x at first target: `/mobile_manipulator_mpc_policy/plan_target_trajectories/state_trajectory[0]/value[0]`
  - Predicted base x (first step): `/mobile_manipulator_mpc_policy/state_trajectory[0]/value[0]`
  - Predicted input base v_x (first step): `/mobile_manipulator_mpc_policy/input_trajectory[0]/value[0]`
  - Flattened input v_x (first step): `/mobile_manipulator_mpc_policy/data[0]/data[0]`

Notes
- rqt_plot shows scalar vs wall‑time; increase index `[i]` to view later horizon points.
- `data[i]/data[j]` mirrors `input_trajectory[i]/value[j]` for this example (8‑D input).
- All trajectories are expressed in the world frame unless otherwise documented.

