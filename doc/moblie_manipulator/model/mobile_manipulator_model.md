# Mobile Manipulator MPC — Model and Targets

## 1. Overview

- The OCS2 mobile manipulator demo selects the kinematic model via `model_information.manipulatorModelType` in the `task.info` file.
- Ridgeback + UR5 uses `WheelBasedMobileManipulator` (`manipulatorModelType=1`).
- Code references:
  - Dynamics selection/setup: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:221`
  - Wheel–base kinematics: `ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp:44`
  - End-effector pose constraint: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:188`
  - Joint‑limit soft constraints: `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:500`

## 2. OCS2 MPC Formulation

For this demo there is a single domain (no mode switches). Using the [OCS2](https://leggedrobotics.github.io/ocs2/getting-started.html) denotation and formulation, the OCP solved by the MPC is

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

- The MPC formulation minimizes over $\mathbf{u}(\cdot)$ on $[t_0,\,t_f]$, then the optimal control sequence $\mathbf{u}(t)$ and the corresponding state forcasted trajectory $\mathbf{x}(t)$ are rollout in a receding-horizon's fashion.
- MPC problem is solved with `SLQ`/`ILQR` (see `ddp` settings in the `task.info` file).

## 3. Specialization to the wheel–based manipulator, *e.g.* Ridgeback + UR5

- State and input
  $$
  \mathbf{x} = \begin{bmatrix}x & y & \theta & \mathbf{q}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{3+n},\qquad
  \mathbf{u} = \begin{bmatrix}v & \omega & \dot{\mathbf{q}}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top} \in \mathbb{R}^{2+n}.
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
  - End‑effector constraint
    - EE pose reference (from `TargetTrajectories`)
      $$
      \boldsymbol{\xi}^{\mathrm{ref}}_{\mathrm{ee}}(t) =
      \begin{bmatrix} \boldsymbol{p}_{\mathrm{ee}}^{\mathrm{ref}}(t) \\
      \boldsymbol{q}_{\mathrm{ee}}^{\mathrm{ref}}(t)\end{bmatrix} \in \mathbb{R}^{7}.
      $$

    - Forward kinematics (FK) maps current system state $\mathbf{x}(t)$ to current EE pose $\boldsymbol{\xi}_{\mathrm{ee}}(\mathbf{x}) = \mathbf{H}_{\mathrm{fk}}(\mathbf{x}) = \big(\boldsymbol{p}_{\mathrm{ee}}(\mathbf{x}),\,\boldsymbol{q}_{\mathrm{ee}}(\mathbf{x})\big)$.
    - Quaternion is then converted into rotation matrix $\boldsymbol{R} = \boldsymbol{R}(\boldsymbol{q}(\mathbf{x})) = \boldsymbol{R}(\mathbf{x}) \in SO(3) \simeq \mathbb{R}^{3\times 3}$.

    - Define end-effector pose error $\mathbf{e} := \big(\mathbf{e}_p,\,\mathbf{e}_o\big)$, where

      $$
      \begin{aligned}
      &\; \mathbf{e}_p = \boldsymbol{p}_{\mathrm{ee}}(\mathbf{x}) - \boldsymbol{p}_{\mathrm{ee}}^{\mathrm{ref}}(t) \in \mathbb{R}^3, \\
      &\; \mathbf{e}_o = \mathrm{Log}\!\left( \boldsymbol{R}_{\mathrm{ee}}^{\mathrm{ref}}(t)^{\top}\, \boldsymbol{R}_{\mathrm{ee}}(\mathbf{x}) \right) \in SO(3) \simeq \mathbb{R}^3.
      \end{aligned}
      $$
  
  - Input (*e.g.* velocity) constraint
    - Input velocity reference (from `TargetTrajectories`), usually set as zero feedforward.
      $$
      \mathbf{u}^{\mathrm{ref}}(t) =
      \begin{bmatrix}v^{\mathrm{ref}} \\ \omega^{\mathrm{ref}} \\ \dot{\mathbf{q}}_{\mathrm{arm}}^{\mathrm{ref}} \end{bmatrix} \in \mathbb{R}^{2+n}.
      $$
    - Define input error $\mathbf{e}_{u} := \mathbf{u}(t) - \mathbf{u}^{\mathrm{ref}}(t)$

  - Inequality constraints $h_i$ (enforced softly via penalties $p(\cdot)$):
    - Self‑collision: $d_i(\mathbf{x}) - d_{\min} \ge 0$ (params `selfCollision.mu`, `selfCollision.delta`).
    - Joint position/velocity limits from URDF and `task.info`.

- Cost functions
  - Stage cost

    $$
    \begin{align*}
    \ell(\mathbf{x},\mathbf{u},t) = \
    & \tfrac{1}{2}\, \Big( \|\mathbf{e}_p\|^2_{\mathbf{Q}_{\mathrm{ee}_p}} + \|\mathbf{e}_o\|^2_{\mathbf{Q}_{\mathrm{ee}_o}} \Big) \quad + \\
    & \tfrac{1}{2}\, \|\mathbf{e}_{u}\|^2_{\mathbf{R}} \quad + \\
    & \sum_i p\big(h_i(\mathbf{x})\big).
    \end{align*}
    $$

  - Terminal cost

    $$
    \phi\big(\mathbf{x}(t_f)\big)
    = \tfrac{1}{2}\, \Big( \|\mathbf{e}_p(t_f)\|^2_{\mathbf{Q}_{\mathrm{ee}_p}^{\mathrm{f}}} + \|\mathbf{e}_o(t_f)\|^2_{\mathbf{Q}_{\mathrm{ee}_p}^{\mathrm{f}}}\Big).
    $$

**Parameter mapping in `task.info`**
- $\mathbf{R}$ → `inputCost.R.*`.
- $\{\mathbf{Q}_{\mathrm{ee}_p},\,\mathbf{Q}_{\mathrm{ee}_o}\}$ are accessed through $\{\mu_p,\mu_o\}$ → `endEffector.muPosition`, `endEffector.muOrientation`.
- $\{\mathbf{Q}_{\mathrm{ee}_p}^{\mathrm{f}},\,\mathbf{Q}_{\mathrm{ee}_o}^{\mathrm{f}}\}$ are accessed through $\{\mu_p^{\mathrm{f}},\mu_o^{\mathrm{f}}\}$ → `finalEndEffector.muPosition`, `finalEndEffector.muOrientation`.
- Penalty params → `selfCollision.*`, `joint*Limits.*`.
- `EndEffectorConstraint` → `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp:299, 320–419`

## 4. Constructing TargetTrajectories (Reference)

- `TargetTrajectories(timeTrajectory, stateTrajectory, inputTrajectory)`: time‑parameterized reference consumed by costs/constraints (via `ReferenceManager`).

- Dual arm mode: concatenate $\begin{bmatrix}\mathbf{p}_L^{\top}& \mathbf{q}_L^{\top}& \mathbf{p}_R^{\top}& \mathbf{q}_R^{\top}\end{bmatrix}^{\top} \in \mathbb{R}^{14}$ per target state.

- See [mobile_manipulator_target.md](../target/mobile_manipulator_target.md) for instructions on setting OCS2 `TargetTrajectories`.

## 5. Sanity Checklist

- Confirm `manipulatorModelType` matches your robot.

- Verify `stateDim`/`inputDim`/`armDim` via `ManipulatorModelInfo` and startup logs.
  - *E.g.* Ridgeback + UR5: `stateDim = 3 + 6 = 9`, `inputDim = 2 + 6 = 8`.

- Remember for wheel‑based: 
  - States $\begin{bmatrix}x& y& \theta& \mathbf{q}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{3+n}$.
  - Inputs $\begin{bmatrix}v& \omega& \dot{\mathbf{q}}_{\mathrm{arm}}^{\top}\end{bmatrix}^{\top}\in \mathbb{R}^{2+n}$.

- Eigen quaternion coefficient order is `[x, y, z, w]`, which is different than its declaration `[w, x, y, z]`. Ensure unit quaternions when packing $\boldsymbol{q}$.
