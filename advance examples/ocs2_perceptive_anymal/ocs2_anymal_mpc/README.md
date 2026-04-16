# OCS2 Anymal MPC

This package provide a mpc example for Anymal quadruped robot. Besides the basic movement like ocs2_legged_robot_ros, this example provided "demo_motion" example.

![image-20240806094724070](../../../.images/anymal_mpc.png)

* build command
```bash
cd ~/ros2_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
export MAKEFLAGS=-j2 && \
export NINJAFLAGS=-j2 && \
colcon build --symlink-install \
--packages-up-to ocs2_anymal_mpc \
--executor sequential --parallel-workers 2 \
--cmake-force-configure \
--cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
. install/setup.bash
```

* launch command
```bash
ros2 launch ocs2_anymal_mpc anymal_c.launch.py
```