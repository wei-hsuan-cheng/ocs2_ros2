Mobile Manipulator MPC: Model And Targets

Overview
- This demo uses the OCS2 mobile manipulator stack. The concrete model and dimensions are selected by `model_information.manipulatorModelType` in the task file.
- In the Ridgeback+UR5 example, the type is `WheelBasedMobileManipulator` (value `1`).
- Code references in this repo:
  - Dynamics selection and problem setup: ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:221
  - Wheel-based dynamics definition: ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp:44
  - End‑effector constraint hookup: ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:188
  - Joint limit soft constraints: ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:500

State And Input Definitions (by model type)
- DefaultManipulator (0)
  - State `x = q_arm ∈ R^n` (arm joint positions)
  - Input `u = qdot_arm ∈ R^n`
  - Dynamics: `ẋ = u`
- FloatingArmManipulator (2)
  - State `x = [q_base(6); q_arm(n)]` (floating XYZ+RPY base + joints)
  - Input `u = qdot_arm ∈ R^n` (base unactuated)
  - Dynamics: `ẋ_base = 0`, `ẋ_arm = u`
- FullyActuatedFloatingArmManipulator (3)
  - State `x = [q_base(6); q_arm(n)]`
  - Input `u = [v_base(6); qdot_arm(n)]`
  - Dynamics: `ẋ = u`
- WheelBasedMobileManipulator (1) — used in the Ridgeback+UR5 demo
  - State `x = [x, y, θ, q_arm(1..n)] ∈ R^{3+n}`
  - Input `u = [v, ω, qdot_arm(1..n)] ∈ R^{2+n}`
  - Kinematic dynamics (unicycle base + joint velocities):
    - `ẋ = v cos θ`
    - `ẏ = v sin θ`
    - `θ̇ = ω`
    - `q̇_arm = qdot_arm`
  - Implementation: ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp:44

How To Inspect Dimensions At Runtime
- The interface exposes `ManipulatorModelInfo`:
  - `interface.getManipulatorModelInfo().stateDim` → total state dimension
  - `interface.getManipulatorModelInfo().inputDim` → total input dimension
  - `interface.getManipulatorModelInfo().armDim` → number of arm joints
- In the Ridgeback+UR5 case: `stateDim = 3 + 6 = 9`, `inputDim = 2 + 6 = 8`.
- You can also see them indirectly in logs at node startup where limits/cost matrices are printed with full vector sizes.

What Does TargetTrajectories Contain?
- `TargetTrajectories` is the time‑parameterized reference used by costs/constraints (via the `ReferenceManager`).
- It is not the system state; its size is defined by the modules that consume it.
  - In this demo, the end‑effector soft constraint reads a 7‑element target per arm: `[px, py, pz, qx, qy, qz, qw]` (Eigen quaternion coeff order).
  - Single arm → 7 elements. Dual arm → 14 elements (left 7 + right 7).
- The input reference inside `TargetTrajectories` (third array) is optional here; we set it to zeros of size `inputDim`.

Where The Target Is Used
- `MobileManipulatorInterface` attaches an `EndEffectorConstraint` that reads the target from the `ReferenceManager` and penalizes the difference between current EE pose and the target pose over time:
  - End‑effector constraint creation: ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:299
  - Single vs dual arm handling and penalties: ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:320–419
- Penalties (weights) are set from the task file under `endEffector` and `finalEndEffector` (`muPosition`, `muOrientation`).
- Additional soft constraints in the problem:
  - Joint position limits (from URDF): `jointPositionLimits.*`
  - Joint velocity limits (task file): `jointVelocityLimits.*`
  - Self‑collision: `selfCollision.*` (relaxed‑barrier penalty)
- Stage cost includes a quadratic input cost `uᵀ R u` (split between base and arm blocks): `inputCost.R.*`.

MPC Formulation (high‑level)
- Continuous kinematics `ẋ = f(x,u)` as above (no second‑order rigid‑body dynamics in this demo).
- Minimize over `u(t)` on `[t, t+T]`:
  - Stage: `∫ (uᵀRu + penalties(endEffector error, self‑collision, limits)) dt`
  - Terminal: penalties on end‑effector final pose
- Solved by SLQ/ILQR per `ddp` settings in the task file.

Constructing TargetTrajectories
- Single arm example (hold at current EE pose):
  - Build one 7‑vector target and a zero input of length `inputDim`.
  - Code pattern (see MobileManipulatorDummyMRT.cpp:310):
    - `const vector_t zeroInput = vector_t::Zero(info.inputDim);`
    - `const TargetTrajectories traj({t0}, {target7}, {zeroInput});`
- Single arm, two waypoints over time (linear interpolation occurs in the constraint):
  - `std::vector<scalar_t> times{t0, t1};`
  - `std::vector<vector_t> states{target7_t0, target7_t1};`
  - `std::vector<vector_t> inputs{zeroInput, zeroInput};`
  - `TargetTrajectories traj(times, states, inputs);`
- Dual arm: concatenate left and right `[pL, qL, pR, qR]` → 14 entries per state in `TargetTrajectories`.

Frame And Quaternion Notes
- Marker and target frame:
  - For `WheelBasedMobileManipulator` the marker frame defaults to `world` (see MobileManipulatorTarget.cpp: getMarkerFrameFromTaskFile).
  - For `DefaultManipulator`, it uses the robot `baseFrame`.
- Quaternion order is Eigen’s coefficient order `[x, y, z, w]`; ensure unit quaternions.

Putting It Together
- To assign a custom end‑effector reference trajectory from your own node:
  - Compute the desired EE pose sequence in the chosen frame (world for wheel‑based).
  - Pack each into a 7‑vector `[p, q]` using Eigen’s `Quaterniond::coeffs()`.
  - Build `TargetTrajectories(times, stateTargets, zeroInputs)` where `zeroInputs[i]` has length `inputDim`.
  - Publish to the MPC target topic (the demo uses the interactive marker to do this for you).

Sanity Checklist
- Confirm `manipulatorModelType` in the task file matches the robot model you intend.
- Verify `inputDim`/`stateDim` with `ManipulatorModelInfo` and the startup logs.
- For wheel‑based, remember inputs are `[v, ω, qdot_arm]` and states are `[x, y, θ, q_arm]`.
