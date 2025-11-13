# Mobile Manipulator MPC — Target (Marker, Twist, and Trajectory Modes)

This document summarizes how the three target modes construct `TargetTrajectories(timeTrajectory, stateTrajectory, inputTrajectory)` and the corresponding math equations. For more details on the model of MPC see [mobile_manipulator_model.md](../model/mobile_manipulator_model.md).

The MPC consumes these via the `ReferenceManager` and evaluates references at query time using linear interpolation.

Reference interpolation behavior:

- `getDesiredState(t)` and `getDesiredInput(t)` perform linear interpolation over the provided samples; a single sample acts as a constant function.
  - Code: `/ocs2_ros2/core/ocs2_core/src/reference/TargetTrajectories.cpp:66-81`.
  - Implementation: `/ocs2_ros2/core/ocs2_core/include/ocs2_core/misc/implementation/LinearInterpolation.h:63-67, 110-120, 120-138`.

## Marker Mode (interactive marker pose tracking)

- Code (single arm): `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorMarkerTarget.cpp:110-129`.
- Code (dual arm): `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorMarkerTarget.cpp:135-157`.

Construction of `TargetTrajectories` (single arm):

- `timeTrajectory = { t0 }`, where `t0 = observation.time`.
- `stateTrajectory = { [ p_ref, q_ref ] }`, with `p_ref ∈ R^3`, `q_ref = [qx, qy, qz, qw]^T ∈ R^4` (Eigen coeff order).
  - Zero-order held pose over the horizon:
    $$
    \boldsymbol{\xi}^{\mathrm{ref}}_{\mathrm{ee}}(t) =
      \begin{bmatrix} \boldsymbol{p}_{\mathrm{ee}}^{\mathrm{ref}}(t) \\
      \boldsymbol{q}_{\mathrm{ee}}^{\mathrm{ref}}(t)\end{bmatrix} \in \mathbb{R}^{7},\quad t \in [t_0, t_0+T].
    $$
- `inputTrajectory = { 0_{m} }`, `m = inputDim`.

Dual arm uses a 14-vector per sample by concatenating left and right 7-vectors.

## Twist Mode (end-effector twist tracking)

- Code (forward propatate trajectory by integrating twist; first-order approximation):
  - `N`, `dt`, and `arrays`: `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTwistTarget.cpp:159-173`.
  - Per-step time/state/input trajectories pushback and integration loop: `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTwistTarget.cpp:177-200`.
  - Delta quaternion from angular rate: `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTwistTarget.cpp:48-57`.

Construction of `TargetTrajectories`:

- `timeTrajectory`
    $$
    t_k = t_0 + k\,\Delta t,\quad \forall k = 0,\ldots,N,\quad N = \lceil T/\Delta t \rceil.$$
  - $T$: forcast horizon of the trajectory by twist integration.
  - $\Delta t$: sampling time of the trajectory.
  - $N$: Number of waypoints in the trajectory.

- `stateTrajectory`
  - Start from current FK pose $(\boldsymbol{p}_0, \boldsymbol{q}_0)$ at the start of each iteration $t_0$ if available.
  - Define $\mathrm{Exp}(\boldsymbol{\omega}\,\Delta t) = \begin{bmatrix} \cos(\tfrac{\theta}{2}) & \; \boldsymbol{u}\,\sin(\tfrac{\theta}{2})^{\top} \end{bmatrix}^{\top}$, with $\theta = \|\boldsymbol{\omega}\|\,\Delta t$ and $\boldsymbol{u} = \boldsymbol{\omega}/\|\boldsymbol{\omega}\|$.
  - Quaternion multiplication is denoted by $\otimes$.
  - Pose propagation in either world or end-effector frame
    - If `twistInWorld = true` (twist expressed in world):
        $$
        \begin{aligned}
        \boldsymbol{p}_{k+1} &= \boldsymbol{p}_k + \boldsymbol{v}\,\Delta t, \\
        \boldsymbol{q}_{k+1} &= \mathrm{Exp}(\boldsymbol{\omega}\,\Delta t) \otimes \boldsymbol{q}_k.
        \end{aligned}
        $$

    - If `twistInWorld = false` (twist expressed in EE frame):
        $$
        \begin{aligned}
        \boldsymbol{p}_{k+1} &= \boldsymbol{p}_k + \boldsymbol{R}(\boldsymbol{q}_k)\,\boldsymbol{v}\,\Delta t, \\
        \boldsymbol{q}_{k+1} &= \boldsymbol{q}_k \otimes \mathrm{Exp}(\boldsymbol{\omega}\,\Delta t).
        \end{aligned}
        $$

- `inputTrajectory`
  - Set as zero vector to disable feedforward.

- Wrapping up
  - `timeTrajectory = [t_0, t_1, ..., t_N]`.
  - `stateTrajectory[k] = [ p_{ref,k}, q_{ref,k} ] \in R^7`.
  - `inputTrajectory[k] = 0_m` (zeros with dimension equal to the system input).

## Trajectory Mode (*e.g.* parametric “eight” in a plane)

- Code (eight-shape trajectory generator): `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTrajectoryTarget.cpp:65-87`.
- Code (wrapping up): `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTrajectoryTarget.cpp:194-209`.

Let $\boldsymbol{a}$ be the unit normal of the plane (parameter `axisX/Y/Z`). Build an orthonormal in‑plane basis $\{\boldsymbol{u},\boldsymbol{v}\}$. With amplitude $A$ and frequency $f$ (s.t. $\omega=2\pi f$), and anchor time $t_\mathrm{start}$, the end‑effector reference is:

$$
\begin{aligned}
\boldsymbol{p}(t) &= \boldsymbol{p}_c \; + \; A\,\boldsymbol{u}\,\sin(\omega\,(t - t_\mathrm{start})) \; + \; \tfrac{A}{2}\,\boldsymbol{v}\,\sin\big(2\,\omega\,(t - t_\mathrm{start})\big), \\
\boldsymbol{q}(t) &= \boldsymbol{q}_0 \quad \text{(constant orientation).}
\end{aligned}
$$

Trajectory arrays over $t_k = t_0 + k\,\Delta t$, $k=0,\ldots,N$.

- Wrapping up
  - `timeTrajectory = [t_0, ..., t_N]`.
  - `stateTrajectory[k] = [ p_{ref,k}, q_{ref,k} ] \in R^7`.
  - `inputTrajectory[k] = 0_m` (zeros per sample).

---

Launch helpers for the three modes live under:

- Marker: `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/launch/include/mobile_manipulator_marker.launch.py`.
- Twist: `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/launch/include/mobile_manipulator_twist.launch.py`.
- Trajectory: `/ocs2_ros2/basic examples/ocs2_mobile_manipulator_ros/launch/include/mobile_manipulator_trajectory.launch.py`.
