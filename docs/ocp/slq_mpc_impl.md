# SLQ-MPC implementation in OCS2 (code walkthrough)

This note complements the derivation in [`docs/ocp/slq_mpc.md`](slq_mpc.md) by mapping the ICRA2016 SLQ / SLQ‑MPC pseudocode to the concrete implementation under `mpc/`.

## 1) Where SLQ‑MPC lives in the codebase

OCS2 splits “MPC loop” from “optimal control solver”:

- **MPC loop + horizon:** `MPC_BASE::run()` computes `finalTime = currentTime + timeHorizon` and calls `calculateController()` [`mpc/ocs2_mpc/src/MPC_BASE.cpp#L45`](../../mpc/ocs2_mpc/src/MPC_BASE.cpp#L45).
- **SLQ‑MPC wrapper:** `GaussNewtonDDP_MPC` selects `SLQ` vs `ILQR` based on settings and calls `ddpPtr_->run(initTime, initState, finalTime)` each MPC tick [`mpc/ocs2_ddp/src/GaussNewtonDDP_MPC.cpp#L8`](../../mpc/ocs2_ddp/src/GaussNewtonDDP_MPC.cpp#L8).
- **Solver core (DDP loop):** `GaussNewtonDDP::runImpl()` implements the iterative loop: LQ approximation → backward pass → controller synthesis → line search / acceptance [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L1005`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L1005).
- **SLQ backward pass (continuous‑time Riccati ODE):** `SLQ` overrides the backward pass and controller synthesis hooks [`mpc/ocs2_ddp/src/SLQ.cpp#L77`](../../mpc/ocs2_ddp/src/SLQ.cpp#L77).

In other words: **SLQ‑MPC = `MPC_BASE` receding horizon + `GaussNewtonDDP` (DDP/iLQR family) configured with `SLQ`**.

## 2) Data structures: paper symbols → code fields

The solver keeps primal (nominal trajectories) and dual (multipliers + value function) in two containers:

- **Nominal trajectory** $ \tau = \{t_k, x_k^n, u_k^n\} $ is `PrimalDataContainer::primalSolution` [`mpc/ocs2_ddp/include/ocs2_ddp/DDP_Data.h#L50`](../../mpc/ocs2_ddp/include/ocs2_ddp/DDP_Data.h#L50).
- **LQ model at each node** (linearized dynamics + quadratized cost + constraints) is `PrimalDataContainer::modelDataTrajectory[k]` [`mpc/ocs2_ddp/include/ocs2_ddp/DDP_Data.h#L50`](../../mpc/ocs2_ddp/include/ocs2_ddp/DDP_Data.h#L50).
- **Value function approximation** $V(t,x)\approx s(t) + S_v(t)^\top \delta x + \tfrac12 \delta x^\top S_m(t)\delta x$ is stored as `DualDataContainer::valueFunctionTrajectory[k]` [`mpc/ocs2_ddp/include/ocs2_ddp/DDP_Data.h#L85`](../../mpc/ocs2_ddp/include/ocs2_ddp/DDP_Data.h#L85).
  - `ScalarFunctionQuadraticApproximation::dfdxx` ↔ $S_m$ / $P$
  - `ScalarFunctionQuadraticApproximation::dfdx` ↔ $S_v$ / $p$
  - `ScalarFunctionQuadraticApproximation::f` ↔ $s$
- **Controller parameterization** matches the paper’s affine feedback law:
  - `LinearController` implements $u(t,x) = K(t)\,x + u_{ff}(t)$ [`core/ocs2_core/include/ocs2_core/control/LinearController.h#L38`](../../core/ocs2_core/include/ocs2_core/control/LinearController.h#L38).
  - OCS2 also carries a **feedforward update direction** `deltaBiasArray_` (the $l(t)$ in the paper’s line search update) [`core/ocs2_core/include/ocs2_core/control/LinearController.h#L109`](../../core/ocs2_core/include/ocs2_core/control/LinearController.h#L109).

When SLQ builds the “unoptimized controller” after the backward pass, it chooses:

- `gainArray_[k]` = $K_k$
- `biasArray_[k]` = $u_k^n - K_k x_k^n$ (so the nominal is reproduced by the controller)
- `deltaBiasArray_[k]` = $l_k$ (feedforward increment direction)

see [`mpc/ocs2_ddp/src/SLQ.cpp#L128`](../../mpc/ocs2_ddp/src/SLQ.cpp#L128).

## 3) Algorithm 2 (SLQ‑MPC) → code path per MPC tick

The ICRA2016 “repeat: acquire state → solve SLQ → apply first control” maps to:

1. **User code calls MPC** with the latest measurement: `MPC_BASE::run(currentTime, currentState)` [`mpc/ocs2_mpc/src/MPC_BASE.cpp#L45`](../../mpc/ocs2_mpc/src/MPC_BASE.cpp#L45).
2. **Horizon selection**: `finalTime = currentTime + timeHorizon_` [`mpc/ocs2_mpc/src/MPC_BASE.cpp#L54`](../../mpc/ocs2_mpc/src/MPC_BASE.cpp#L54).
3. **Solve OCP on that horizon**: `GaussNewtonDDP_MPC::calculateController()` optionally resets the solver on cold start and then runs the DDP solver [`mpc/ocs2_ddp/src/GaussNewtonDDP_MPC.cpp#L24`](../../mpc/ocs2_ddp/src/GaussNewtonDDP_MPC.cpp#L24).
4. **Apply the policy externally** (OCS2 exposes the computed `PrimalSolution::controllerPtr_` and trajectories via solver APIs; applying only the first control is outside these core files).

Warm‑start vs cold‑start is controlled by `mpc::Settings::coldStart_` [`mpc/ocs2_mpc/include/ocs2_mpc/MPC_Settings.h#L40`](../../mpc/ocs2_mpc/include/ocs2_mpc/MPC_Settings.h#L40) and enforced in [`mpc/ocs2_ddp/src/GaussNewtonDDP_MPC.cpp#L24`](../../mpc/ocs2_ddp/src/GaussNewtonDDP_MPC.cpp#L24).

## 4) Inside one solver call: the SLQ / DDP iteration loop

`GaussNewtonDDP::runImpl()` is the implementation of “repeat SLQ iterations until convergence / iteration limit”:

### 4.1 Initial rollout (warm start + forward simulation)

This corresponds to Algorithm 1 “simulate the system dynamics” to obtain a nominal $\tau$.

- Set pointers to desired trajectories used by the cost terms (`targetTrajectoriesPtr`) [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L1015`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L1015).
- Build the initial nominal primal:
  - Try to **rollout the previous controller** (warm start) [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L856`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L856), [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L394`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L394).
  - Else, try to **reuse previous trajectories** (state/input) [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L868`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L868).
  - Finally, **fill the remainder** of the horizon using the `Initializer` rollout [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L459`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L459).
- The actual forward simulation call is `rolloutTrajectory()` → `rollout.run(...)` [`mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L125`](../../mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L125).

### 4.2 LQ approximation (linearize dynamics + quadratize cost)

This is Algorithm 1 “Linearize dynamics along $\tau$” and “Quadratize cost along $\tau$”.

- `GaussNewtonDDP::approximateOptimalControlProblem()` calls the virtual hook `approximateIntermediateLQ(...)` over all nodes [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L667`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L667).
- For SLQ, that hook is implemented as:
  - loop over time indices and call `ocs2::approximateIntermediateLQ(...)` to fill `ModelData` [`mpc/ocs2_ddp/src/SLQ.cpp#L77`](../../mpc/ocs2_ddp/src/SLQ.cpp#L77).
- The actual derivative computation happens in the approximator:
  - dynamics linearization: `problem.dynamicsPtr->linearApproximation(...)` (`dfdx`, `dfdu`) [`core/ocs2_oc/src/approximate_model/LinearQuadraticApproximator.cpp#L47`](../../core/ocs2_oc/src/approximate_model/LinearQuadraticApproximator.cpp#L47)
  - cost quadratization: `modelData.cost = approximateCost(...)` (`dfdx`, `dfdu`, `dfdxx`, `dfdux`, `dfduu`) [`core/ocs2_oc/src/approximate_model/LinearQuadraticApproximator.cpp#L52`](../../core/ocs2_oc/src/approximate_model/LinearQuadraticApproximator.cpp#L52)

Event times (`jumpMapLinearApproximation` + event cost) and final time cost are approximated in the same function family and wired in `approximateOptimalControlProblem()` [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L678`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L678).

### 4.3 Backward pass (solve continuous‑time Riccati equations)

In the paper (Algorithm 1) this is the backward recursion of $P(t), p(t)$. In OCS2‑SLQ it is implemented as a **backward ODE integration** of the continuous‑time Riccati equations:

1. **Project / normalize the LQ problem** and compute per‑node “Riccati modification” terms:
   - `SLQ::solveSequentialRiccatiEquations()` fills `projectedModelDataTrajectory` and `riccatiModificationTrajectory` via `computeProjectionAndRiccatiModification(...)` [`mpc/ocs2_ddp/src/SLQ.cpp#L174`](../../mpc/ocs2_ddp/src/SLQ.cpp#L174), [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L759`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L759).
   - `projectLQ(...)` performs the change of variables $u = P_u \tilde{u} + P_x x + u_0$ to handle state‑input equality constraints (and to normalize input cost) [`mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L143`](../../mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L143).
2. **Solve the Riccati ODE**:
   - `GaussNewtonDDP::solveSequentialRiccatiEquationsImpl(...)` partitions the horizon (multi‑threaded) and calls `riccatiEquationsWorker(...)` [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L530`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L530).
   - SLQ’s worker sets pointers and integrates backward in time [`mpc/ocs2_ddp/src/SLQ.cpp#L209`](../../mpc/ocs2_ddp/src/SLQ.cpp#L209).

The continuous‑time Riccati dynamics are in `ContinuousTimeRiccatiEquations::computeFlowMapSLQ()`:

- It interpolates LQ coefficients $A(t), B(t), Q(t), R(t), P(t), q(t), r(t)$ from `projectedModelDataTrajectory` and the “modification” terms $\Delta Q, \Delta G$ [`mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L168`](../../mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L168).
- It computes the projected gains/increments (the continuous‑time analogue of $K(t)=-H^{-1}G$, $l(t)=-H^{-1}g$) and then assembles $\dot S_m, \dot S_v, \dot s$ (Riccati ODE) [`mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L210`](../../mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L210).

Backward integration details:

- SLQ integrates over a **normalized negative time axis** (so forward integration in `z` corresponds to backward in real time) [`mpc/ocs2_ddp/src/SLQ.cpp#L231`](../../mpc/ocs2_ddp/src/SLQ.cpp#L231), [`mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L142`](../../mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L142).
- Hybrid event times are handled via `computeJumpMap()` → `riccatiTransversalityConditions(...)` [`mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L124`](../../mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L124), [`mpc/ocs2_ddp/include/ocs2_ddp/riccati_equations/RiccatiTransversalityConditions.h#L39`](../../mpc/ocs2_ddp/include/ocs2_ddp/riccati_equations/RiccatiTransversalityConditions.h#L39).

### 4.4 Controller synthesis (build $K(t)$ and $l(t)$)

After the backward pass, `GaussNewtonDDP::calculateController()` fills a `LinearController` by calling the virtual `calculateControllerWorker()` for each time index [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L608`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L608).

For SLQ the worker is:

- `SLQ::calculateControllerWorker(...)` which:
  - forms the projected feedback/feedforward terms from $(S_m,S_v)$ and the projected LQ coefficients [`mpc/ocs2_ddp/src/SLQ.cpp#L152`](../../mpc/ocs2_ddp/src/SLQ.cpp#L152)
  - maps them back to the original input coordinates (using the projection matrix `Qu`) and writes:
    - `gainArray_[k]` (feedback)
    - `biasArray_[k]` (nominal feedforward consistent with $(x_k^n,u_k^n)$)
    - `deltaBiasArray_[k]` (feedforward increment direction) [`mpc/ocs2_ddp/src/SLQ.cpp#L160`](../../mpc/ocs2_ddp/src/SLQ.cpp#L160)

This matches the paper’s update law:
$$
u(t) = u^n(t) + \alpha\,l(t) + K(t)\,(x(t)-x^n(t)).
$$

### 4.5 Line search / step acceptance (choose $\alpha$)

`GaussNewtonDDP` delegates “apply $\alpha$, rollout, accept/reject” to a `SearchStrategyBase` selected from settings [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L89`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L89).

For the paper‑style SLQ line search, the relevant implementation is `LineSearchStrategy`:

- It builds candidate controllers by scaling only the feedforward increment:
  - `incrementController(stepLength, unoptimized, controller)` keeps `gainArray_` fixed and sets `bias = bias + α·deltaBias` [`mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L303`](../../mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L303).
- It rolls out each candidate controller and computes the merit/cost [`mpc/ocs2_ddp/src/search_strategy/LineSearchStrategy.cpp#L84`](../../mpc/ocs2_ddp/src/search_strategy/LineSearchStrategy.cpp#L84).
- It uses an Armijo‑style condition to accept the largest improving step length [`mpc/ocs2_ddp/src/search_strategy/LineSearchStrategy.cpp#L235`](../../mpc/ocs2_ddp/src/search_strategy/LineSearchStrategy.cpp#L235).

The solver calls the strategy from `takePrimalDualStep()` and updates dual variables if the rollout is accepted [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L930`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L930).

## 5) Practical notes vs the ICRA2016 pseudocode

- **Continuous‑time vs discrete‑time notation:** the paper’s Algorithm 1 is written in discrete time, but OCS2‑SLQ integrates a continuous‑time Riccati ODE backward (and uses ODE rollout forward) [`mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L168`](../../mpc/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp#L168).
- **Policy lag compensation / state prediction:** the “policy lag” step in Algorithm 2 is not part of `MPC_BASE`; it is typically handled by the caller (predict state forward, then call `MPC_BASE::run()`).
- **Terminal cost from infinite‑horizon LQR:** OCS2 provides `continuous_time_lqr::solve(...)` to compute $K_\infty$ and $S_\infty$ around an equilibrium, which can be used to form the terminal cost $H$ as suggested in the paper [`mpc/ocs2_ddp/include/ocs2_ddp/ContinuousTimeLqr.h#L52`](../../mpc/ocs2_ddp/include/ocs2_ddp/ContinuousTimeLqr.h#L52).
- **Constraints:** beyond the unconstrained pseudocode, OCS2 supports state‑input equality constraints via projection (`projectLQ`) and uses a merit function with penalty terms in the line search loop [`mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L143`](../../mpc/ocs2_ddp/src/DDP_HelperFunctions.cpp#L143), [`mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L516`](../../mpc/ocs2_ddp/src/GaussNewtonDDP.cpp#L516).

