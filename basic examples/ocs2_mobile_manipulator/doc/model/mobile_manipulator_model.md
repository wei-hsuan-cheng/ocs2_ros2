# Mobile Manipulator MPC — Model and Targets

## Overview

- The OCS2 mobile manipulator demo selects the kinematic model via `model_information.manipulatorModelType` in the `task.info` file.
- Ridgeback + UR5 uses `WheelBasedMobileManipulator` (`manipulatorModelType=1`).
- Code references:
  - Dynamics selection/setup: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:221`
  - Wheel–base kinematics: `ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp:44`
  - End-effector pose constraint: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:188`
  - Joint‑limit soft constraints: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:500`

## State and Input (by model type)

- DefaultManipulator (0)
  - State and input

  $$
  \mathbf{x} = \mathbf{q}_{\text{arm}} \in \mathbb{R}^{n},\qquad
  \mathbf{u} = \dot{\mathbf{q}}_{\text{arm}} \in \mathbb{R}^{n}.
  $$

  - Dynamics

  $$
  \dot{\mathbf{x}} = \mathbf{u}.
  $$

- **WheelBasedMobileManipulator (1)**, *e.g.* Ridgeback + UR5
  - State and input

  $$
  \mathbf{x} = \begin{bmatrix}x & y & \theta & \mathbf{q}_{\text{arm}}^{\top}\end{bmatrix}^{\top} \in \mathbb{R}^{3+n},\qquad
  \mathbf{u} = \begin{bmatrix}v & \omega & \dot{\mathbf{q}}_{\text{arm}}^{\top}\end{bmatrix}^{\top} \in \mathbb{R}^{2+n}.
  $$

  - Kinematics (unicycle base + joint rates)

  $$
  \dot{\mathbf{x}} = \mathbf{f}(\mathbf{x},\mathbf{u})
  = \begin{bmatrix}
  v\cos\theta \\
  v\sin\theta \\
  \omega \\
  \dot{\mathbf{q}}_{\text{arm}}
  \end{bmatrix}.
  $$

  - Implementation: `ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp:44`

- FloatingArmManipulator (2)
  - State and input (floating base Euler ZYX + arm)

  $$
  \mathbf{x} = \begin{bmatrix}\mathbf{q}_{\text{base}} \\ \mathbf{q}_{\text{arm}}\end{bmatrix} \in \mathbb{R}^{6+n},\qquad
  \mathbf{u} = \dot{\mathbf{q}}_{\text{arm}} \in \mathbb{R}^{n}.
  $$

  - Dynamics

  $$
  \dot{\mathbf{q}}_{\text{base}} = \mathbf{0},\qquad
  \dot{\mathbf{q}}_{\text{arm}} = \mathbf{u}.
  $$

- FullyActuatedFloatingArmManipulator (3)
  - State and input

  $$
  \mathbf{x} = \begin{bmatrix}\mathbf{q}_{\text{base}} \\ \mathbf{q}_{\text{arm}}\end{bmatrix},\qquad
  \mathbf{u} = \begin{bmatrix}\mathbf{v}_{\text{base}} \\ \dot{\mathbf{q}}_{\text{arm}}\end{bmatrix}.
  $$

  - Dynamics (first‑order)

  $$
  \dot{\mathbf{x}} = \mathbf{u}.
  $$

## Inspecting Dimensions at Runtime

- `ManipulatorModelInfo` provides sizes:
  - `stateDim`, `inputDim`, `armDim`.
- *E.g.* Ridgeback + UR5: `stateDim = 3 + 6 = 9`, `inputDim = 2 + 6 = 8`.

## TargetTrajectories (reference)

- Time‑parameterized reference consumed by costs/constraints (via `ReferenceManager`).
- Not the full system state; its size is defined by modules that consume it.
  - In this demo, the EE soft constraint reads a 7‑vector per arm

$$
\mathbf{r}^{\text{ee}}(t)
= \begin{bmatrix}\mathbf{p}^{\top}(t) & \mathbf{q}^{\top}(t)\end{bmatrix}^{\top}
\in \mathbb{R}^{7},\qquad \mathbf{q} = \begin{bmatrix}q_x& q_y& q_z& q_w\end{bmatrix}^{\top}.
$$

- For two arms, concatenate left and right to obtain 14 entries.
- The input reference track is set to zero of length `inputDim`.

## Where the Target is Used

- `EndEffectorConstraint` penalizes end-effector pose error along the horizon:
  - Creation/attachment: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:299, 320–419`
- Weights from task file: `endEffector.muPosition`, `endEffector.muOrientation`, and terminal counterparts.
- Additional soft constraints: joint limits, self‑collision.
- Input stage cost

  $$
  \int \mathbf{u}^{\top} \mathbf{R} \, \mathbf{u} \, dt,
  $$

  with base/arm blocks configured under `inputCost.R`.

## MPC Formulation (high‑level)

- Continuous dynamics

  $$
  \dot{\mathbf{x}} = \mathbf{f}(\mathbf{x},\mathbf{u}).
  $$

- Minimize over $\mathbf{u}(\cdot)$ on $[t,\,t+T]$:

  $$
  \min_{\mathbf{u}(\cdot)} \; \int_{t}^{t+T} \Big( \mathbf{u}^{\top}\mathbf{R}\,\mathbf{u}
  \; + \; \text{penalties}(\text{EE error},\, \text{limits},\, \text{self‑collision}) \Big)\, dt
  \; + \; \Phi(\text{EE error at } t+T).
  $$

- Solved with SLQ/ILQR (see `ddp` settings in the task file).

## Constructing TargetTrajectories

- Single waypoint (hold current EE pose): build one 7‑vector and a zero input (length `inputDim`).
  - Pattern (see `MobileManipulatorDummyMRT.cpp:310`):
    `TargetTrajectories({t0}, {target7}, {zeroInput})`.
- Two waypoints: provide two times and two 7‑vectors (zero inputs for both); linear interpolation is handled internally.
- Dual arm: concatenate $\begin{bmatrix}\mathbf{p}_L^{\top}& \mathbf{q}_L^{\top}& \mathbf{p}_R^{\top}& \mathbf{q}_R^{\top}\end{bmatrix}^{\top} \in \mathbb{R}^{14}$ per target state.

## Frames and Quaternions

- For wheel‑based, references and markers are in the `world` frame (see `MobileManipulatorTarget.cpp` helper).
- Eigen quaternion coefficient order is `[x, y, z, w]`, which is different than its declaration `[w, x, y, z]`. Ensure unit quaternions when packing $\mathbf{q}$.

## Sanity Checklist

- Confirm `manipulatorModelType` matches your robot.
- Verify `stateDim`/`inputDim` via `ManipulatorModelInfo` and startup logs.
- Remember for wheel‑based: 
  - Inputs $\begin{bmatrix}v& \omega& \dot{\mathbf{q}}_{\text{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{2+n}$.
  - States $\begin{bmatrix}x& y& \theta& \mathbf{q}_{\text{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{3+n}$.

## OCS2 Denotation — Optimal Control Problem

For this demo there is a single domain (no mode switches). Using the [OCS2 denotation](https://leggedrobotics.github.io/ocs2/getting-started.html), the OCP solved by the MPC is

$$
\begin{aligned}
\min_{\mathbf{u}(\cdot)}\; & \; \phi\big(\mathbf{x}(t_f)\big)\; + \int_{t_0}^{t_f} \ell\big(\mathbf{x}(t),\mathbf{u}(t),t\big)\,dt \\
\text{s.t.}\;
& \; \mathbf{x}(t_0)=\mathbf{x}_0, \\
& \; \dot{\mathbf{x}}(t) = \mathbf{f}\big(\mathbf{x}(t),\mathbf{u}(t),t\big), \\
& \; \mathbf{g}_1\big(\mathbf{x}(t),\mathbf{u}(t),t\big) = \mathbf{0}, \\
& \; \mathbf{g}_2\big(\mathbf{x}(t),t\big) = \mathbf{0}, \\
& \; \mathbf{h}\big(\mathbf{x}(t),\mathbf{u}(t),t\big) \ge \mathbf{0}.
\end{aligned}
$$

Specialization to the wheel–based manipulator:

- State and input

  $$
  \mathbf{x} = \begin{bmatrix}x & y & \theta & \mathbf{q}_{\text{arm}}^{\top}\end{bmatrix}^{\top},\qquad
  \mathbf{u} = \begin{bmatrix}v & \omega & \dot{\mathbf{q}}_{\text{arm}}^{\top}\end{bmatrix}^{\top}.
  $$

- Dynamics (first‑order kinematics)

  $$
  \dot{\mathbf{x}} =
  \begin{bmatrix}
  v\cos\theta \\
  v\sin\theta \\
  \omega \\
  \dot{\mathbf{q}}_{\text{arm}}
  \end{bmatrix}
  \;=\; \mathbf{f}(\mathbf{x},\mathbf{u}).
  $$

- End‑effector reference (from `TargetTrajectories`):

  $$
  \mathbf{r}^{\text{ee}}(t)
  = \begin{bmatrix}\mathbf{p}^{\top}_{\text{ref}}(t) & \mathbf{q}^{\top}_{\text{ref}}(t)\end{bmatrix}^{\top} \in \mathbb{R}^{7}.
  $$

  - Forward kinematics yields $\mathbf{y}^{\text{ee}}(\mathbf{x}) = \big(\mathbf{p}_{\text{ee}}(\mathbf{x}),\,\mathbf{R}_{\text{ee}}(\mathbf{x})\big)$. Define pose error

    $$
    \begin{aligned}
    &\; \mathbf{e}_p = \mathbf{p}_{\text{EE}}(\mathbf{x}) - \mathbf{p}_{\text{ref}}(t) \in \mathbb{R}^3, \\
    &\; \mathbf{e}_R = \mathrm{Log}\!\left( \mathbf{R}_{\text{ref}}(t)^{\top}\, \mathbf{R}_{\text{EE}}(\mathbf{x}) \right) \in SO(3) \simeq \mathbb{R}^3.
    \end{aligned}
    $$

- Running cost

$$
\ell(\mathbf{x},\mathbf{u},t)
= \tfrac{1}{2}\, \mathbf{u}^{\top}\mathbf{R}\,\mathbf{u}
\; + \; \tfrac{1}{2}\,\mu_p\, \|\mathbf{e}_p\|^2
\; + \; \tfrac{1}{2}\,\mu_o\, \|\mathbf{e}_R\|^2
\; + \; \sum_i p\!\big(h_i(\mathbf{x})\big).
$$

- Terminal cost

$$
\phi\big(\mathbf{x}(t_f)\big)
= \tfrac{1}{2}\,\mu_p^{\mathrm{f}}\, \|\mathbf{e}_p(t_f)\|^2
\; + \; \tfrac{1}{2}\,\mu_o^{\mathrm{f}}\, \|\mathbf{e}_R(t_f)\|^2.
$$

- Inequalities $h_i$ (enforced softly via penalties $p(\cdot)$):
  - Self‑collision: $d_i(\mathbf{q}) - d_{\min} \ge 0$ (params `selfCollision.mu`, `selfCollision.delta`).
  - Joint position/velocity limits from URDF and `task.info`.

Parameter mapping:

- $\mathbf{R}$ → `inputCost.R.*`.
- $\mu_p,\mu_o$ → `endEffector.muPosition`, `endEffector.muOrientation`.
- $\mu_p^{\mathrm{f}},\mu_o^{\mathrm{f}}$ → `finalEndEffector.muPosition`, `finalEndEffector.muOrientation`.
- Penalty params → `selfCollision.*`, `joint*Limits.*`.
