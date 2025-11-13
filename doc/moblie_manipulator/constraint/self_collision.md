# Self‑Collision Constraint (Mobile Manipulator)

## Overview

- Goal: keep a minimum distance between selected robot links while tracking an end‑effector (EE) pose.
- Implemented as a soft state inequality constraint added to the OCP solved by OCS2 (SLQ/ILQR).
- Geometry and distances are computed with Pinocchio + hpp‑fcl; constraint value and Jacobian are provided to OCS2 each iteration.


## Where It Is Built

- Constraint factory: `MobileManipulatorInterface::getSelfCollisionConstraint(...)`
  - Path: `ocs2_ros2_ws/src/ocs2_ros2/basic examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`
  - Reads settings from `task.info` under `selfCollision` (pairs, minimumDistance, mu, delta).
  - Creates a `PinocchioGeometryInterface` from URDF and the configured link/object pairs.
  - Chooses either analytic (`SelfCollisionConstraint`) or AutoDiff (`SelfCollisionConstraintCppAd`) form, then wraps it as a soft constraint with a penalty function.


## Geometry, Value and Jacobian

- For each configured collision pair $i$, compute the signed closest distance $d_i(q)$ and closest points $p^1_i(q),\,p^2_i(q)$.
- Minimum distance threshold $d_{\min}$ from `task.info`.
- Inequality per pair:

$$
f_i(q) \,=\, d_i(q) - d_{\min} \,\ge\, 0,\qquad f(q) = [f_1,\ldots,f_m]^\top.
$$

- Linearization (analytic form): point Jacobians are built from the parent joint Jacobians translated to the contact point (Pinocchio local‑world aligned frame)

$$
J_p \,=\, J_{\text{pos}} \,-
\,[p_{\text{offset}}]_\times\, J_{\text{rot}}.
$$

Let $n_i(q)$ be the unit vector along $(p^2_i-p^1_i)$ (or its opposite if $d_i\le 0$). Then

$$
\frac{\partial f_i}{\partial q}(q)\,=\, n_i(q)^{\top}\bigl(J_{p^2}(q)-J_{p^1}(q)\bigr).
$$

- Code: `ocs2_self_collision/src/SelfCollision.cpp` and `SelfCollisionConstraint.cpp`.
- Mapping to OCS2 state: `PinocchioStateInputMapping` maps $\partial f/\partial q$ to the current OCS2 state ordering (base + arm). See `SelfCollisionConstraint::getLinearApproximation(...)`.


## Current Penalty (why you see drift)

- The factory wraps the constraint with a relaxed log‑barrier penalty (soft constraint): `RelaxedBarrierPenalty(\mu,\delta)`. For a single inequality $h\ge 0$ (here $h=f_i(q)$):

$$
p(h)\,=\,
\begin{cases}
-\mu\,\ln h, & h>\delta,\\[4pt]
-\mu\,\ln\delta\, +\, \mu\,\dfrac{1}{2}\Bigl(\bigl(\tfrac{h-2\delta}{\delta}\bigr)^2-1\Bigr), & \text{otherwise}.
\end{cases}
$$

- Property: $p'(h)<0$ for all $h>0$. Even when safely away from collision, the cost still decreases as $h$ grows, so the optimizer keeps increasing distances. This explains the observed drift once the EE goal is met.


## Task File Knobs

- `task.info` → `selfCollision` block (example fields):
  - `activate` (bool), `collisionLinkPairs` / `collisionObjectPairs`, `minimumDistance`, `mu`, `delta`.
  - Paths: `.../config/<robot>/task.info` (mobile manipulator examples provide several). Changing `mu` lowers the drift but does not remove the bias of the log penalty.

## How To Make It Minimally‑Intervening
The constraint should produce (near) zero gradient when distances are safely above the threshold, only reacting close to contact.

## Option A — Augmented squared‑hinge penalty (zero gradient when safe)
- Replace the penalty with `augmented::SlacknessSquaredHingePenalty({scale, stepSize})`.
- For a single inequality $h\ge 0$ with multiplier $\lambda$ and scale $\rho$:

$$
p(h,\lambda)\,=\,\frac{1}{2\rho}\Big(\max\{0,\,\lambda-\rho h\}^2-\lambda^2\Big),
\quad
\nabla_h p(h,\lambda)\,=\,\begin{cases}
0, & h\ge \lambda/\rho,\\
-(\lambda-\rho h)/\rho, & h<\lambda/\rho.
\end{cases}
$$

- Hence the gradient is zero when safely away from the boundary, producing minimally‑intervening behavior.
- Code change (single place):
  - File: `MobileManipulatorInterface.cpp` (in `getSelfCollisionConstraint`), replace
    - `auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});`
    - with e.g. `auto penalty = augmented::SlacknessSquaredHingePenalty::create({scale, stepSize});`
- Tuning: start with `scale ≈ 10–100`, `stepSize ≈ 1.0`.

## Option B — Modified relaxed barrier (reduced bias far from boundary)
- Use `augmented::ModifiedRelaxedBarrierPenalty({scale, relaxation, stepSize})`.
- For $h\ge 0$, multiplier $\lambda$, scale $\rho$, define

$$
p(h,\lambda)\,=\,\frac{\lambda^2}{\rho}\,\psi\!\left(\frac{\rho h}{\lambda}\right),
$$

where $\psi(\cdot)$ is a shifted, quadratically‑relaxed log barrier. Far from the boundary, both value and gradient decay faster than $-\ln h$, reducing bias to keep increasing $h$.
- Same replacement site as Option A.

## Option C — Custom dead‑zone penalty
- Implement a simple ReLU‑squared penalty with a user dead‑zone $\varepsilon$:

$$
p(h)\,=\,\begin{cases}
0, & h\ge \varepsilon,\\[2pt]
\tfrac{1}{2}w\,(h-\varepsilon)^2, & h<\varepsilon.
\end{cases}
$$
- Implement a new `PenaltyBase` and instantiate it in place of the current penalty (same factory location).

## Where Pairs and Markers Come From
- Pairs are read from `task.info` (`collisionLinkPairs`, `collisionObjectPairs`) and turned into a Pinocchio geometry model.
- RViz markers (green lines with distances) are published by `GeometryInterfaceVisualization` inside the dummy visualization node:
  - `.../basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyVisualization.cpp`

## Quick Recipe: Switch to minimally‑intervening behavior
1) Edit penalty in `MobileManipulatorInterface::getSelfCollisionConstraint(...)` as per Option A or B.
2) (Optional) Expose new penalty parameters via `task.info` (e.g., `selfCollision.scale`, `selfCollision.stepSize`) and load them alongside `minimumDistance`.
3) Rebuild and run. With hinge penalty, the robot stops moving once EE tracks the goal and all `h_i` are safely above the activation threshold.

## Files of Interest
- Constraint value/Jacobian (analytic):
  - `robotics/ocs2_pinocchio/ocs2_self_collision/src/SelfCollision.cpp`
  - `robotics/ocs2_pinocchio/ocs2_self_collision/src/SelfCollisionConstraint.cpp`
- AutoDiff variant:
  - `robotics/ocs2_pinocchio/ocs2_self_collision/src/SelfCollisionConstraintCppAd.cpp`
- Factory & penalty selection:
  - `basic examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`
- Penalties:
  - Relaxed barrier: `core/ocs2_core/include/ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h`
  - Augmented squared hinge: `core/ocs2_core/include/ocs2_core/penalties/augmented/SlacknessSquaredHingePenalty.h`
  - Modified relaxed barrier: `core/ocs2_core/include/ocs2_core/penalties/augmented/ModifiedRelaxedBarrierPenalty.h`

## Mathematical Summary
- Constraint per pair:

$$
f_i(q) = d_i(q) - d_{\min} \ge 0.
$$

- Distance Jacobian:

$$
\frac{\partial f_i}{\partial q} = n_i^{\top}(J_{p^2}-J_{p^1}),\qquad
J_p = J_{\text{pos}} - [p_{\text{offset}}]_\times J_{\text{rot}}.
$$

- Soft constraint cost:

$$
\sum_i p\big(f_i(q)\big),
$$

with $p(\cdot)$ chosen (relaxed barrier, augmented hinge, modified relaxed barrier, or custom dead‑zone).

## Notes
- The current relaxed barrier will continuously reduce cost as distances grow; expect drift when the EE cost is already small.
- Augmented penalties eliminate or reduce this bias and are the recommended choice for minimally‑intervening avoidance.
