# Mobile Manipulator MPC — Model and Targets

## Overview

- The OCS2 mobile manipulator demo selects the kinematic model via `model_information.manipulatorModelType` in the `task.info` file.
- Ridgeback + UR5 uses `WheelBasedMobileManipulator` (`manipulatorModelType=1`).
- Code references:
  - Dynamics selection/setup: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:221`
  - Wheel–base kinematics: `ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp:44`
  - End-effector pose constraint: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:188`
  - Joint‑limit soft constraints: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:500`

## OCS2 MPC Formulation

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

Minimize over $\mathbf{u}(\cdot)$ on $[t_0,\,t_f]$; solved with SLQ/ILQR (see `ddp` settings in the `task.info` file).

**Specialization to the wheel–based manipulator, *e.g.* Ridgeback + UR5**

- State and input
  $$
  \mathbf{x} = \begin{bmatrix}x & y & \theta & \mathbf{q}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top},\qquad
  \mathbf{u} = \begin{bmatrix}v & \omega & \dot{\mathbf{q}}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top}.
  $$

- System dynamics (first‑order kinematic-model; unicycle base + joint velocities)
  $$
  \dot{\mathbf{x}} =
  \begin{bmatrix}
  v\cos\theta \\
  v\sin\theta \\
  \omega \\
  \dot{\mathbf{q}}_{\mathrm{arm}}
  \end{bmatrix}
  \;=\; \mathbf{f}(\mathbf{x},\mathbf{u}).
  $$

- Penalties and constraints (EE error, joint limits, self-collision)
  - End‑effector reference (from `TargetTrajectories`):
    $$
    \mathbf{r}^{\mathrm{ee}}(t)
    = \begin{bmatrix}\boldsymbol{p}^{\top}_{\mathrm{ref}}(t) & \boldsymbol{q}^{\top}_{\mathrm{ref}}(t)\end{bmatrix}^{\top} \in \mathbb{R}^{7}.
    $$

    - Forward kinematics (FK) yields $\mathbf{y}^{\mathrm{ee}}(\mathbf{x}) = \mathbf{H}_{\mathrm{fk}}(\mathbf{x}) = \big(\boldsymbol{p}_{\mathrm{ee}}(\mathbf{x}),\,\boldsymbol{q}_{\mathrm{ee}}(\mathbf{x})\big)$. Quaternion is then converted into rotation matrix $\boldsymbol{R} = \boldsymbol{R}(\boldsymbol{q}) \in SO(3) \simeq \mathbb{R}^{3\times 3}$.
    
    - Define end-effector pose error $\mathbf{e} = \big(\mathbf{e}_p,\,\mathbf{e}_o\big)$, where

      $$
      \begin{aligned}
      &\; \mathbf{e}_p = \boldsymbol{p}_{\text{ee}}(\mathbf{x}) - \boldsymbol{p}_{\mathrm{ref}}(t) \in \mathbb{R}^3, \\
      &\; \mathbf{e}_o = \mathrm{Log}\!\left( \boldsymbol{R}_{\mathrm{ref}}(t)^{\top}\, \boldsymbol{R}_{\mathrm{ee}}(\mathbf{x}) \right) \in SO(3) \simeq \mathbb{R}^3.
      \end{aligned}
      $$

  - Running cost

    $$
    \ell(\mathbf{x},\mathbf{u},t)
    = \tfrac{1}{2}\, \mathbf{u}^{\top}\mathbf{R}\,\mathbf{u}
    \; + \; \tfrac{1}{2}\,\mu_p\, \|\mathbf{e}_p\|^2
    \; + \; \tfrac{1}{2}\,\mu_o\, \|\mathbf{e}_o\|^2
    \; + \; \sum_i p\!\big(h_i(\mathbf{x})\big).
    $$

  - Terminal cost

    $$
    \phi\big(\mathbf{x}(t_f)\big)
    = \tfrac{1}{2}\,\mu_p^{\mathrm{f}}\, \|\mathbf{e}_p(t_f)\|^2
    \; + \; \tfrac{1}{2}\,\mu_o^{\mathrm{f}}\, \|\mathbf{e}_o(t_f)\|^2.
    $$

  - Inequality constraints $h_i$ (enforced softly via penalties $p(\cdot)$):
    - Self‑collision: $d_i(\mathbf{x}) - d_{\min} \ge 0$ (params `selfCollision.mu`, `selfCollision.delta`).
    - Joint position/velocity limits from URDF and `task.info`.

**Parameter mapping**

- $\mathbf{R}$ → `inputCost.R.*`.
- $\mu_p,\mu_o$ → `endEffector.muPosition`, `endEffector.muOrientation`.
- $\mu_p^{\mathrm{f}},\mu_o^{\mathrm{f}}$ → `finalEndEffector.muPosition`, `finalEndEffector.muOrientation`.
- Penalty params → `selfCollision.*`, `joint*Limits.*`.
- `EndEffectorConstraint` → `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:299, 320–419`

## Constructing TargetTrajectories (Reference)

- `TargetTrajectories(timeTrajectory, stateTrajectory, inputTrajectory)`: time‑parameterized reference consumed by costs/constraints (via `ReferenceManager`).
  
- Single waypoint (hold current EE pose): build one 7‑vector and a zero input (length `inputDim`).
  - Pattern (see `MobileManipulatorDummyMRT.cpp:310`):
    `TargetTrajectories({t0}, {target7}, {zeroInput})`.

- Two waypoints: provide two times and two 7‑vectors (zero inputs for both); linear interpolation is handled internally.

- Dual arm mode: concatenate $\begin{bmatrix}\mathbf{p}_L^{\top}& \mathbf{q}_L^{\top}& \mathbf{p}_R^{\top}& \mathbf{q}_R^{\top}\end{bmatrix}^{\top} \in \mathbb{R}^{14}$ per target state.

## Sanity Checklist

- Confirm `manipulatorModelType` matches your robot.

- Verify `stateDim`/`inputDim`/`armDim` via `ManipulatorModelInfo` and startup logs.
  - *E.g.* Ridgeback + UR5: `stateDim = 3 + 6 = 9`, `inputDim = 2 + 6 = 8`.

- Remember for wheel‑based: 
  - States $\begin{bmatrix}x& y& \theta& \mathbf{q}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{3+n}$.
  - Inputs $\begin{bmatrix}v& \omega& \dot{\mathbf{q}}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{2+n}$.

- Eigen quaternion coefficient order is `[x, y, z, w]`, which is different than its declaration `[w, x, y, z]`. Ensure unit quaternions when packing $\boldsymbol{q}$.

