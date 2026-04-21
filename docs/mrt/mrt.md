# MRT in OCS2

This note explains what MRT does in OCS2, how it co-works with the MPC policy, and how the current `mpc_controller` / `mobile_manipulator_mpc` integration communicates in this workspace.

The concrete example used below is the current `dual_ur5` setup:

- [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:31>)
- [`dual_ur5.launch.py`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/launch/dual_ur5.launch.py:27>)

## 1. Short answer

- `MPC` is the optimizer. It solves a finite-horizon optimal control problem and produces a policy over the horizon.
- `MRT` is the fast execution layer. It receives the latest available policy, updates it when a new one arrives, and evaluates or rolls that policy forward at the controller rate.

In block-diagram language, MRT sits between the solver and the robot:

```text
reference / targets / mode schedule
               |
               v
      +----------------------+
      |   Reference Manager  |
      +----------------------+
               |
               v
      +----------------------+
      |     MPC solver       |
      |  receding horizon    |
      |  slow update loop    |
      +----------------------+
               |
               | latest policy pi*(t, x)
               v
      +----------------------+
      |        MRT           |
      | fast policy executor |
      | and short rollout    |
      +----------------------+
               |
               | command u(t)
               v
      +----------------------+
      |    robot / plant     |
      +----------------------+
               |
               v
         measured state
```

MRT is not the optimizer itself. MRT is the layer that keeps using the latest available optimized policy at a faster rate than the solver updates.

## 2. The three relevant time scales

There are three different time scales in the current stack.

### 2.1 Solver grid

This is the knot spacing inside the optimized policy.

For `dual_ur5`:

- `ddp.timeStep = 0.01 s` in [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:47>)
- `sqp.dt = 0.01 s` in [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:66>)

So the optimized policy is represented on a `10 ms` grid.

### 2.2 MPC update rate

This is how often a new policy is requested / produced.

For `dual_ur5`:

- `mpc.mpcDesiredFrequency = 125 Hz` in [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:100>)
- one MPC refresh every `8 ms`

### 2.3 MRT execution rate

This is how often the currently active policy is executed.

For `dual_ur5`:

- `mpc.mrtDesiredFrequency = 250 Hz` in [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:101>)
- one MRT execution step every `4 ms`

So in this example, MRT runs twice as fast as the policy refresh rate, and both run faster than the policy knot spacing.

## 3. Timing view

With the current `dual_ur5` numbers:

- solver grid `dt = 10 ms`
- `mpcFreq = 125 Hz` -> `8 ms`
- `mrtFreq = 250 Hz` -> `4 ms`

the timing picture looks like this:

```text
time [ms]:     0----4----8---12---16---20---24---28---32

solver knots:  K0---------K1---------K2---------K3---------
               0         10         20         30

mrt ticks:     m0   m1   m2   m3   m4   m5   m6   m7   m8
query time:    0    4    8    12   16   20   24   28   32

mpc refresh:   R0        R1        R2        R3
               |         |         |         |
policy arrive: ----P0----      ----P1----      ----P2----
               (latency)        (latency)        (latency)

active policy:   none -> P0 -> P0 -> P0 -> P1 -> P1 -> P2 -> ...
```

Important consequence:

- MRT does not always use the first input knot of the policy.
- If time has already advanced before a new policy arrives, MRT keeps executing the latest available policy at later query times.
- So MRT may use what is informally the "later part" of the current policy, typically by interpolation in time and then either direct evaluation or short rollout.

## 4. What MRT actually stores and executes

The core MRT implementation is [`MRT_BASE`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:52>) with the main execution logic in [`MRT_BASE.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp:36>).

Its responsibilities are:

- keep an active policy and a buffered incoming policy
- swap the buffered policy into the active slot when `updatePolicy()` succeeds
- expose the current command metadata, primal solution, and performance indices
- optionally evaluate the policy at a query time/state
- optionally roll the policy forward over one short step if a rollout object was attached

In other words, MRT is a policy consumer and executor.

### 4.1 `updatePolicy()`

[`updatePolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp:160>) does not solve MPC.

It only checks whether a new policy has already arrived in the internal buffer and, if so, swaps it into the active slot.

Conceptually:

```text
ROS callback receives new policy
    -> decode message
    -> move policy into MRT buffer

controller fast loop calls updatePolicy()
    -> swap buffered policy into active policy
```

### 4.2 `evaluatePolicy()`

[`evaluatePolicy(t, x, ...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp:98>) samples the active policy at time `t`.

It does two things:

- computes the control input from the controller stored in the policy
- interpolates the nominal state trajectory at that time

It does not integrate the system dynamics.

Conceptually:

```text
policy at time t  -> interpolate controller data
                  -> compute u(t, x)
                  -> interpolate x_nom(t)
```

### 4.3 `rolloutPolicy()`

[`rolloutPolicy(t, x, dt, ...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp:122>) is stronger than `evaluatePolicy()`.

It:

- starts from the current state `x` at time `t`
- uses the active policy as the closed-loop controller
- forward-simulates only over the short interval `[t, t + dt]`
- returns the end-of-step state and input

So in MRT, "rollout one step" means:

```text
simulate from now to now + mrt_dt
```

not:

```text
simulate from now to the full MPC horizon
```

The full-horizon rollout belongs to the MPC solver side, not the MRT fast execution side.

## 5. What policy MRT executes

MRT does not have two different execution modes. The important switch is the type of policy produced by MPC.

### 5.1 Feedforward-only policy

If `useFeedbackPolicy = false`, MPC produces a `FeedforwardController`, and MRT eventually evaluates [`FeedforwardController::computeInput()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/core/ocs2_core/src/control/FeedforwardController.cpp:79>).

Then MRT uses a time-varying feedforward input:

```text
u(t) = interpolate(uff(t))
```

This is the closest case to a plain interpolator.

### 5.2 Linear feedback policy

If `useFeedbackPolicy = true`, MPC produces a `LinearController`, and MRT eventually evaluates [`LinearController::computeInput()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/core/ocs2_core/src/control/LinearController.cpp:66>).

In the current OCS2 implementation, MRT evaluates:

```text
u(t, x) = uff(t) + K(t) x
```

where both `uff(t)` and `K(t)` are interpolated at the current query time.

So the correct mental model is:

- the whole policy is refreshed at `mpcFreq`
- the current policy is queried at `mrtFreq`
- with feedback enabled, both the feedforward and feedback terms are executed at the MRT rate

For the current `dual_ur5` task, both:

- `ddp.useFeedbackPolicy = true`
- `sqp.useFeedbackPolicy = true`

both in [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:52>) and [`task.info`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/config/dual_ur5/task.info:78>)

so MRT is usually executing an interpolated linear feedback policy, not only a feedforward trajectory.

## 6. Current controller behavior in this workspace

In the current integration, [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:432>) creates an [`MRT_ROS_Interface`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/include/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h:51>), attaches a rollout object, and launches a dedicated bridge node:

```text
OCS2Controller
  -> MRT_ROS_Interface(robotTopicPrefix(robot_name))
  -> initRollout(interface.getRollout())
  -> launchNodes(mrt_bridge_node)
```

Concrete lines:

- construct bridge: [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:434>)
- attach rollout: [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:435>)
- create bridge node: [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:440>)
- launch ROS endpoints: [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:442>) and [`MRT_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:248>)

The bridge node is named `<controller_node_name>_mrt_bridge`.

### 6.1 Realtime loop

In realtime mode, each controller tick does:

```text
1. build current observation
2. mrt_->setCurrentObservation(obs)
3. mrt_->updatePolicy()
4. if no policy yet: hold
5. choose query time t_req
6. mrt_->rolloutPolicy(t_req, obs.state, mrt_dt, ...)
7. apply command to robot
```

Current code path:

- update entry: [`OCS2Controller::update()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:739>)
- realtime step: [`runRealtimeLoopStep()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:755>)
- spin ROS callbacks: [`mrt_->spinMRT()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:745>)
- publish observation: [`mrt_->setCurrentObservation(obs)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:758>)
- swap newest policy: [`updatePolicyTimed()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:687>) and [`mrt_->updatePolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:692>)
- short rollout execution: [`mrt_->rolloutPolicy(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:783>)
- apply filtered command: [`applyFilteredRolloutCommand(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:784>)

So the controller is using MRT as a fast policy rollout/execution block.

### 6.2 Synchronized loop

In synchronized mode, the controller adds one extra rule:

- at MPC boundaries, it tries to wait for a fresh policy whose start time matches the boundary
- if that policy is late, it temporarily holds
- after the wait timeout, it can continue with the latest available policy

So synchronized mode is a stronger timing contract on top of the same MRT machinery.

Current code path:

- synchronized step: [`runSynchronizedLoopStep()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:797>)
- boundary observation publish: [`mrt_->setCurrentObservation(obs)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:809>)
- freshness check: [`policyIsFreshForTime(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:718>)
- short rollout execution: [`mrt_->rolloutPolicy(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:853>)
- pre-publish next boundary observation: [`mrt_->setCurrentObservation(next_obs)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:865>)

## 7. MRT is not only an interpolator

It is fair to say that MRT can behave like an interpolator in the feedforward-only case.

But in general MRT is broader than that:

- with `FeedforwardController`, MRT interpolates `u(t)`
- with `LinearController`, MRT interpolates `uff(t)` and `K(t)` and evaluates a feedback law
- with `rolloutPolicy()`, MRT also forward-simulates the dynamics over one control step

So the best summary is:

```text
plain interpolator  ⊂  MRT
```

MRT includes interpolation as one part of policy execution, but it can also execute feedback and perform a short forward rollout.

## 8. Communication architecture in the current workspace

The current stack uses a mix of:

- in-process C++ method calls
- ROS 2 pub/sub
- ROS 2 services
- ROS 2 actions

There is no shared-memory shortcut between the controller process and the external MPC node at the application level. Their observation/policy exchange is done through ROS 2 messages via [`MRT_ROS_Interface`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/include/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h:51>) and [`MPC_ROS_Interface`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:324>).

### 8.1 High-level communication diagram

```text
user / script / RViz / BT
        |
        | ROS 2 actions / topics
        v
+------------------------------+
| upper application layer      |
|                              |
| - mmbt                       |
| - mpc_teleop                 |
| - mpc_cartesian_planner      |
+------------------------------+
        |
        | ROS 2 reference topics:
        |   /<robot>_mpc_target
        |   /<robot>_base_mpc_target
        |   /<robot>_joint_mpc_target
        |   /<robot>_frame_mpc_target
        |   /<robot>_mode_schedule
        |   /<robot>/env_obstacles
        v
+------------------------------+         local method calls
| mpc_controller               | ----------------------------------------------+
| OCS2Controller               |                                               |
| - odom + joint feedback      |                                               |
| - buildObservation()         |                                               |
+------------------------------+                                               |
        ^                                                                     local
        | robot feedback                                                       |
        |                                                                      v
        |                                        +------------------------------+
        |                                        | mrt_bridge node              |
        |                                        | MRT_ROS_Interface            |
        |                                        | - encode obs -> _observation |
        |                                        | - recv _policy into buffer   |
        |                                        +------------------------------+
        |                                                       ^
        | hardware cmds / ros2_control                          |
        v                                                       | ROS 2 pub/sub/service
robot / Mujoco / plant                                          |
                                                                v
                                              +-----------------------------------------+
                                              | mobile_manipulator_mpc node             |
                                              | MPC_ROS_Interface + solver              |
                                              | - decode _observation                   |
                                              | - run mpc_.run(...)                     |
                                              | - flatten/publish _policy               |
                                              +-----------------------------------------+
```

Concrete observation path:

- robot/base feedback enters the controller through the odom subscriber in [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:481>) and arm joint state handles that are read in [`buildObservation()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:998>)
- the controller assembles the OCS2 `SystemObservation` in [`buildObservation()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:998>)
- the controller hands it to MRT through [`mrt_->setCurrentObservation(obs)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:758>) and, at sync boundaries, [`mrt_->setCurrentObservation(next_obs)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:865>)
- `MRT_ROS_Interface` encodes that observation to `ocs2_msgs/msg/MpcObservation` in [`createObservationMsg()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/common/RosMsgConversions.cpp:34>) and [`setCurrentObservation()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:76>)
- the bridge publishes `/<robotName>_mpc_observation` in [`publisherWorkerThread()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:97>)
- `MPC_ROS_Interface` receives that topic in [`launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:324>), decodes it with [`readObservationMsg()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/common/RosMsgConversions.cpp:56>), and runs the solver in [`mpcObservationCallback()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:230>) and [`mpc_.run(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:247>)

Concrete policy path:

- after `mpc_.run(...)`, the MPC node copies the latest solver result into its publish buffer in [`copyToBuffer()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:206>)
- `MPC_ROS_Interface` flattens the OCS2 policy, including `mode_schedule`, in [`createMpcPolicyMsg()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:85>)
- the MPC node publishes `/<robotName>_mpc_policy` in [`publisherWorker()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:177>) and the publisher endpoint is created in [`launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:335>)
- `MRT_ROS_Interface` subscribes to that topic in [`launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:248>), decodes it in [`readPolicyMsg()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:119>), and buffers it in [`mpcPolicyCallback()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:203>)
- the controller then swaps the buffered policy into the active MRT policy with [`mrt_->updatePolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:692>) and [`MRT_BASE::updatePolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp:160>)
- once active, the controller executes it through [`mrt_->rolloutPolicy(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:783>) or [`mrt_->rolloutPolicy(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:853>)

### 8.2 What is local and what is ROS 2

#### Local, same process

Inside `mpc_controller`:

- `OCS2Controller` calls `mrt_->spinMRT()`
- `OCS2Controller` calls `mrt_->setCurrentObservation(...)`
- `OCS2Controller` calls `mrt_->updatePolicy()`
- `OCS2Controller` calls `mrt_->evaluatePolicy(...)` or `mrt_->rolloutPolicy(...)`

These are ordinary C++ calls in the controller process. See:

- [`mrt_->spinMRT()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:745>)
- [`mrt_->setCurrentObservation(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:758>)
- [`mrt_->updatePolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:692>)
- [`mrt_->rolloutPolicy(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:783>)

Inside `mobile_manipulator_mpc`:

- `MPC_ROS_Interface` calls `mpc_.run(...)`
- `MPC_ROS_Interface` copies the solver result into its publish buffer
- `RosReferenceManager` forwards received mode schedules and target trajectories into the local reference manager

These are also ordinary C++ calls in the MPC process. See:

- create reference manager decorator: [`MobileManipulatorMpcNode.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/mpc/MobileManipulatorMpcNode.cpp:136>)
- subscribe reference-manager ROS inputs: [`MobileManipulatorMpcNode.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/mpc/MobileManipulatorMpcNode.cpp:138>)
- run MPC on observation callback: [`MPC_ROS_Interface::mpcObservationCallback()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:230>)
- call solver: [`mpc_.run(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:247>)
- copy solver result into publish buffer: [`copyToBuffer(...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:206>)
- subscribe mode schedule and target trajectory into the reference manager: [`RosReferenceManager::subscribe()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/synchronized_module/RosReferenceManager.cpp:48>)

#### ROS 2 transport

Between the MRT bridge and the MPC node:

- topic `/<robotName>_mpc_observation`
  - type: `ocs2_msgs/msg/MpcObservation`
  - direction: controller -> MPC
- topic `/<robotName>_mpc_policy`
  - type: `ocs2_msgs/msg/MpcFlattenedController`
  - direction: MPC -> controller
- service `/<robotName>_mpc_reset`
  - type: `ocs2_msgs/srv/Reset`
  - direction: controller -> MPC

So yes: `mrt_bridge` is the ROS 2 communication layer for MRT/MPC exchange.

Concrete ROS 2 endpoint setup:

- MRT observation publisher: [`MRT_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:253>)
- MRT policy subscriber: [`MRT_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:258>)
- MRT reset client: [`MRT_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:266>)
- MRT publishes observation: [`MRT_ROS_Interface::setCurrentObservation()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:76>)
- MRT decodes incoming policy: [`MRT_ROS_Interface::readPolicyMsg()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:119>)
- MRT policy callback moves data into the buffer: [`MRT_ROS_Interface::mpcPolicyCallback()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mrt/MRT_ROS_Interface.cpp:203>)
- MPC observation subscriber: [`MPC_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:328>)
- MPC policy publisher: [`MPC_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:335>)
- MPC reset service: [`MPC_ROS_Interface::launchNodes()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:340>)
- MPC flattens the outgoing policy: [`MPC_ROS_Interface::createMpcPolicyMsg()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/mpc/MPC_ROS_Interface.cpp:85>)

It is not only a naming detail. It is the object that opens the ROS publisher, subscriber, and reset client/server needed to connect the controller and the external MPC node.

### 8.3 Current upper-layer interfaces

The upper application layer uses ROS 2 actions and topics.

#### Actions

- `/<robotName>/teleop/execute`
  - provided by `mpc_teleop`
- `/<robotName>/trajectory_tracking/execute_frame_relation_motion`
  - provided by `mpc_cartesian_planner`

`mmbt` acts as a client of these actions in the current workspace.

Concrete code:

- teleop action default name and server: [`teleop_action_server.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_teleop/src/teleop_action_server.cpp:201>) and [`teleop_action_server.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_teleop/src/teleop_action_server.cpp:292>)
- trajectory-tracking action default name and server: [`trajectory_tt_action_server.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_cartesian_planner/src/trajectory_tt_action_server.cpp:96>) and [`trajectory_tt_action_server.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_cartesian_planner/src/trajectory_tt_action_server.cpp:245>)
- `mmbt` registers the corresponding action clients: [`MobileManipulatorBehaviroTree.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/MobileManipulatorBehaviroTree.cpp:213>) and [`MobileManipulatorBehaviroTree.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/MobileManipulatorBehaviroTree.cpp:221>)

#### Reference topics

The application layer publishes:

- `/<robotName>_mpc_target`
- `/<robotName>_base_mpc_target`
- `/<robotName>_joint_mpc_target`
- `/<robotName>_frame_mpc_target`
- `/<robotName>_mode_schedule`
- `/<robotName>/env_obstacles`

These are then consumed by:

- `mobile_manipulator_mpc`
  - through `RosReferenceManager`
  - and through the `MobileManipulatorReferenceManager`-specific subscribers in `MobileManipulatorMpcNode.cpp`
- `mpc_controller`
  - through its local `MobileManipulatorReferenceManager` subscriptions in `OCS2Controller.cpp`
  - except for `/<robotName>_mode_schedule`, which is not directly subscribed by `OCS2Controller`

Concrete code:

- teleop publishes `*_mpc_target`, `*_base_mpc_target`, `*_frame_mpc_target`, and `*_mode_schedule` in [`teleop_action_server.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_teleop/src/teleop_action_server.cpp:263>)
- trajectory tracking publishes `/<robotName>_mode_schedule` in [`trajectory_tt_action_server.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_cartesian_planner/src/trajectory_tt_action_server.cpp:189>)
- trajectory publisher owns the target reference topic publishers in [`trajectory_publisher.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_cartesian_planner/src/trajectory_publisher.cpp:27>)
- `mmbt` publishes mode schedule in [`SwitchMPCModeScheduleBehavior.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/SwitchMPCModeScheduleBehavior.cpp:103>) and [`SwitchMPCModeScheduleBehavior.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/SwitchMPCModeScheduleBehavior.cpp:191>)
- `mmbt` publishes environment obstacles in [`UpdateStaticEnvObstaclesBehavior.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/UpdateStaticEnvObstaclesBehavior.cpp:32>) and [`UpdateStaticEnvObstaclesBehavior.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/UpdateStaticEnvObstaclesBehavior.cpp:310>)
- MPC node subscribes mode schedule through [`RosReferenceManager::subscribe()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/synchronized_module/RosReferenceManager.cpp:48>)
- MPC node creates the decorator and activates its subscriptions in [`MobileManipulatorMpcNode.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/mpc/MobileManipulatorMpcNode.cpp:136>)
- MPC node subscribes base/joint/frame-relation/env-obstacle side channels in [`MobileManipulatorMpcNode.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/mpc/MobileManipulatorMpcNode.cpp:142>)
- controller side subscribes base/joint/frame-relation/env-obstacle side channels in [`OCS2Controller.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mpc_controller/src/control/OCS2Controller.cpp:514>)

Important note about `mode_schedule`:

- the main MPC subscription to `/<robotName>_mode_schedule` is in [`RosReferenceManager.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/synchronized_module/RosReferenceManager.cpp:48>)
- the lifecycle-node overload is in [`RosReferenceManager.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/src/synchronized_module/RosReferenceManager.cpp:71>)
- `OCS2Controller` does not currently have its own direct `mode_schedule` subscriber
- `mmbt` also creates a temporary subscriber to the same topic to verify the echoed mode schedule in [`SwitchMPCModeScheduleBehavior.cpp`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/mobile_manipulator_behaviortree/mmbt/src/SwitchMPCModeScheduleBehavior.cpp:183>)

That duplication is intentional in the current stack:

- the MPC node needs the references for optimization
- the controller side keeps its local reference state aligned as well, especially for local rollout, command application, and monitoring/diagnostics

## 9. Current topic and transport summary

| Interface | Transport | Producer | Consumer | Purpose |
| --- | --- | --- | --- | --- |
| `/<robotName>/teleop/execute` | ROS 2 action | user / BT client | `mpc_teleop` | start/stop teleop sessions |
| `/<robotName>/trajectory_tracking/execute_frame_relation_motion` | ROS 2 action | user / BT client | `mpc_cartesian_planner` | trajectory-tracking goals |
| `/<robotName>_mode_schedule` | ROS 2 topic | teleop / planner / BT | MPC node via `RosReferenceManager`, plus optional `mmbt` echo subscriber | hybrid mode schedule |
| `/<robotName>_mpc_target` | ROS 2 topic | teleop / planner | MPC node via `RosReferenceManager` | primary target trajectories |
| `/<robotName>_base_mpc_target` | ROS 2 topic | teleop / planner | MPC node, controller-side ref mgr | base target stream |
| `/<robotName>_joint_mpc_target` | ROS 2 topic | planner | MPC node, controller-side ref mgr | joint target stream |
| `/<robotName>_frame_mpc_target` | ROS 2 topic | teleop / planner | MPC node, controller-side ref mgr | frame-relation target stream |
| `/<robotName>/env_obstacles` | ROS 2 topic | BT / environment publisher | MPC node, controller-side ref mgr | obstacle updates |
| `/<robotName>_mpc_observation` | ROS 2 topic | MRT bridge | MPC node | current observation for solver |
| `/<robotName>_mpc_policy` | ROS 2 topic | MPC node | MRT bridge | latest flattened policy |
| `/<robotName>_mpc_reset` | ROS 2 service | MRT bridge | MPC node | reset solver / initial target |

## 10. Method cheat sheet

For the current MRT flow, the most important methods are:

| Method | Meaning |
| --- | --- |
| [`setCurrentObservation(obs)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:79>) | publish the latest measured observation toward the MPC node |
| [`spinMRT()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/robotics/ocs2_ros_interfaces/include/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h:83>) | process ROS callbacks so new policy messages are received |
| [`updatePolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:144>) | swap the newest buffered policy into the active policy |
| [`initialPolicyReceived()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:73>) | tell whether any policy has ever arrived |
| [`getPolicy()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:103>) | access the active `PrimalSolution` |
| [`getCommand()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:87>) | access command metadata associated with the active policy |
| [`getPerformanceIndices()`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:95>) | access the active policy's performance indices |
| [`initRollout(rollout)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:109>) | attach the rollout object used by `rolloutPolicy()` |
| [`evaluatePolicy(t, x, ...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:120>) | sample the policy at one time/state, without integrating dynamics |
| [`rolloutPolicy(t, x, dt, ...)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:133>) | execute a short forward rollout from `t` to `t + dt` |
| [`resetMpcNode(target)`](</home/os-weihsuan.cheng/src/wei-hsuan-cheng/ocs2_ros2/mpc/ocs2_mpc/include/ocs2_mpc/MRT_BASE.h:68>) | reset the MPC node through the reset service |

## 11. Practical interpretation

If you only remember one picture, use this:

```text
upper layer publishes targets and mode schedule
                |
                v
      MPC solves horizon problem at mpcFreq
                |
                v
      MRT keeps executing the latest policy at mrtFreq
                |
                v
      robot sees fast commands, not raw solver iterations
```

And if you only remember one distinction, use this:

- `evaluatePolicy()` = sample the current policy
- `rolloutPolicy()` = simulate one short closed-loop step under that policy

That is the core of how MRT co-works with the MPC policy in the current OCS2 mobile manipulator stack.
