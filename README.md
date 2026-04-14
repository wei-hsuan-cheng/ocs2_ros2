# OCS2_ROS2 Toolbox

## 1. Summary

OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), it was refactored to be compatible with ROS 2 and modern cmake. (Forked from [`legubiao/ocs2_ros2`](https://github.com/legubiao/ocs2_ros2))

### Version History

### 2025.11 (forked repo [`wei-hsuan-cheng/ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2))

**Multiple Target Mode for Mobile Manipulator**
- Marker target mode: interactive marker for end-effector pose tracking.
- Twist target mode: end-effector twist tracking via pose forward propagation.
- Trajectory target mode: end-effector trajectory tracking (time-parameterized path).
  
**Fixed small bugs**

### 2025.08 (original repo [`legubiao/ocs2_ros2`](https://github.com/legubiao/ocs2_ros2))

### Tested Platforms

* Intel Nuc X15 (i7-11800H):
    * Ubuntu 22.04 ROS 2 Humble  (WSL2 included)
    * Ubuntu 24.04 ROS 2 Jazzy   (WSL2 included)
* Lenovo P16v (i7-13800H):
    * Ubuntu 24.04 ROS 2 Jazzy
* Jetson Orin Nano
    * Ubuntu 22.04 ROS 2 Humble (JetPack 6.1)
* VM on MacBook Pro with M2 (arm64)
  * Ubuntu 22.04 ROS 2 Humble

## 2. Installation

### 2.1 Prerequisites

The OCS2 library is written in C++17. It is tested under Ubuntu with library versions as provided in the package sources.

Tested system and ROS 2 version:

* Ubuntu 24.04 ROS 2 Jazzy
* Ubuntu 22.04 ROS 2 Humble

### 2.2 Dependencies

* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

> **Note:** Latest version used pinocchio from ros source to simplified install steps. If you install pinocchio from robot-pkgs, you can uninstall it by
> ```bash
> sudo apt remove robotpkg-*
> ```

### 2.3 Clone Repositories

* Create a new workspace or clone the project to your workspace
    ```bash
    cd ~
    mkdir -p ros2_ws/src
    ```

* Clone the repository (with all submodules in `.gitmodules`)
    ```bash
    cd ~/ros2_ws/src
    git clone --recursive https://github.com/wei-hsuan-cheng/ocs2_ros2.git -b humble
    ```

    If you forgot `--recursive`, you can run:
    ```bash
    cd ~/ros2_ws/src/ocs2_ros2
    git submodule update --init --recursive
    ```

* `rosdep`
    ```bash
    cd ~/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

## 3. Basic Examples

This section contains basic examples for the OCS2 library.

### 3.1 [Double Integrator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#double-integrator)

<details>
<summary>🎯 Click to expand Double Integrator example</summary>

* build
    ```bash
    cd ~/ros2_ws
    export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    export MAKEFLAGS=-j2 && \
    export NINJAFLAGS=-j2 && \
    colcon build --symlink-install \
    --packages-up-to ocs2_double_integrator_ros \
    --executor sequential --parallel-workers 2 \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
    ```
* run
    ```bash
    ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
    ```

https://github.com/user-attachments/assets/581d03ff-43e4-49c9-8f47-a0ce491b585c

</details>

### 3.2 [Cartpole](https://leggedrobotics.github.io/ocs2/robotic_examples.html#cartpole)

<details>
<summary>🛒 Click to expand Cartpole example</summary>

* build
    ```bash
    cd ~/ros2_ws
    export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    export MAKEFLAGS=-j2 && \
    export NINJAFLAGS=-j2 && \
    colcon build --symlink-install \
    --packages-up-to ocs2_cartpole_ros \
    --executor sequential --parallel-workers 2 \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
    ```
* run
    ```bash
    ros2 launch ocs2_cartpole_ros cartpole.launch.py
    ```

https://github.com/user-attachments/assets/7fe0fe18-3ad5-47dd-9fe2-be90413c2f2f

</details>

### 3.3 [Ballbot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#ballbot)

<details>
<summary>🏀 Click to expand Ballbot example</summary>

* build
    ```bash
    cd ~/ros2_ws
    export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    export MAKEFLAGS=-j2 && \
    export NINJAFLAGS=-j2 && \
    colcon build --symlink-install \
    --packages-up-to ocs2_ballbot_ros \
    --executor sequential --parallel-workers 2 \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
    ```
* run
    ```bash
    # Launch demo
    ros2 launch ocs2_ballbot_ros ballbot_ddp.launch.py
    # Send target pose to ballbot [x, y, yaw] [m, deg] 
    ros2 run ocs2_ballbot_ros ballbot_target
    ```

https://github.com/user-attachments/assets/c87966b8-525f-4592-a54f-cfaed458a6f2

</details>

### 3.4 [Quadrotor](https://leggedrobotics.github.io/ocs2/robotic_examples.html#quadrotor)

<details>
<summary>🚁 Click to expand Quadrotor example</summary>

* build
    ```bash
    cd ~/ros2_ws
    export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    export MAKEFLAGS=-j2 && \
    export NINJAFLAGS=-j2 && \
    colcon build --symlink-install \
    --packages-up-to ocs2_quadrotor_ros \
    --executor sequential --parallel-workers 2 \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
    ```
* run
    ```bash
    ros2 launch ocs2_quadrotor_ros quadrotor.launch.py
    ```

https://github.com/user-attachments/assets/aed3173f-a6e6-4499-ae8c-d101bedc5222

</details>

### 3.5 [Mobile Manipulator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#mobile-manipulator)

<details>
<summary>🦾 Click to expand Mobile Manipulator example</summary>

* build
    ```bash
    cd ~/ros2_ws
    export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    export MAKEFLAGS=-j2 && \
    export NINJAFLAGS=-j2 && \
    colcon build --symlink-install \
    --packages-up-to ocs2_mobile_manipulator_ros \
    --executor sequential --parallel-workers 2 \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
    ```

* run Mabi-Mobile
    ```bash
    ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch.py
    ```

    https://github.com/user-attachments/assets/c71f6123-fa3a-4b72-a60f-5509b8c25413

* run Kinova
    ```bash
    source ~/ros2_ws/install/setup.bash
    # Jaco2
    ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_j2n6.launch.py
    ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_j2n7.launch.py
    # Gen3 Lite
    ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_lite.launch.py
    ```
* run Franka Panda
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_mobile_manipulator_ros franka.launch.py
    ```

    https://github.com/user-attachments/assets/bab14b46-486e-46dc-a268-bd63616d1010

* run Willow Garage PR2
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_mobile_manipulator_ros pr2.launch.py
    ```

    https://github.com/user-attachments/assets/100aae62-9e80-487b-89cf-ea6a97ef2505

* run UR-5
    ```bash
    source ~/ros2_ws/install/setup.bash
    # Marker/Twist/Trajecoty target modes
    ros2 launch ocs2_mobile_manipulator_ros ur5_marker.launch.py solver:=ddp
    ros2 launch ocs2_mobile_manipulator_ros ur5_twist.launch.py solver:=ddp
    ros2 launch ocs2_mobile_manipulator_ros ur5_trajectory.launch.py solver:=ddp
    # solver:=ddp, sqp
    ```

* run Clearpath Ridgeback with UR-5
    ```bash
    source ~/ros2_ws/install/setup.bash
    # Marker/Twist/Trajecoty target modes
    ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5_marker.launch.py solver:=ddp
    ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5_twist.launch.py solver:=ddp
    ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5_trajectory.launch.py solver:=ddp
    # solver:=ddp, sqp
    ```

</details>

### 3.6 [Legged Robot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot)

<details>
<summary>🐕 Click to expand Legged Robot example</summary>

* build
    ```bash
    cd ~/ros2_ws
    export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    export MAKEFLAGS=-j2 && \
    export NINJAFLAGS=-j2 && \
    colcon build --symlink-install \
    --packages-up-to ocs2_legged_robot_ros \
    --executor sequential --parallel-workers 2 \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
    ```
* run legged robot mpc
    ```bash
    ros2 launch ocs2_legged_robot_ros legged_robot_ddp.launch.py
    # Solvers: _ddp, _sqp, _ipm
    ```

* Set gait command and reference motion
    ```bash
    # Gai command
    ros2 run ocs2_legged_robot_ros legged_robot_gait_command --ros-args \
    -p gaitCommandFile:=<path_to_ws>/install/ocs2_legged_robot/share/ocs2_legged_robot/config/command/gait.info

    # Reference motion
    ros2 run ocs2_legged_robot_ros legged_robot_target --ros-args \
    -p referenceFile:=<path_to_ws>/install/ocs2_legged_robot/share/ocs2_legged_robot/config/command/reference.info
    ```

https://github.com/user-attachments/assets/d29551b7-2ac7-428d-9605-f782193bcaf2

</details>

## 4. Advanced Examples

[![](http://i1.hdslb.com/bfs/archive/a53bab50141165eb452aa0763a9a5b9a51a7ca67.jpg)](https://www.bilibili.com/video/BV1gSHLe3EEv/)

### 4.1 [Perceptive Locomotion](advance%20examples/ocs2_perceptive_anymal/)

![perceptive_side](.images/perception_side.png)

![perceptive_hurdles](.images/perception_hurdles.png)

### 4.2 [RaiSim Simulation](advance%20examples/ocs2_raisim/)

![raisim](.images/raisim.png)

![raisim_rviz](.images/raisim_rviz.png)

### 4.3 [MPC-Net](advance%20examples/ocs2_mpcnet/)

## 5. Related Projects

* [`fiveages-sim/robot_descriptions`](https://github.com/fiveages-sim/robot_descriptions): More robot configs for OCS2_ROS2
* [`fiveages-sim/arms_ros2_control`](https://github.com/fiveages-sim/arms_ros2_control): Mobile manipulator controller based on OCS2_ROS2
* [`legubiao/quadruped_ros2_control`](https://github.com/legubiao/quadruped_ros2_control): Quadruped controller based on OCS2_ROS2