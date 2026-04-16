# OCS2 Anymal Models

Since the [anymal_c package](https://github.com/ANYbotics/anymal_c_simple_description) doesn't have ros2 version yet, please don't used the "camel" option.

This package provide a visualization of the quadruped robot, in the rviz you can adjust the parameter and check robot configuration.

![image-20240806093456198](../../../.images/anymal_model.png)

* build command
```bash
cd ~/ros2_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
export MAKEFLAGS=-j2 && \
export NINJAFLAGS=-j2 && \
colcon build --symlink-install \
--packages-up-to ocs2_anymal_models \
--executor sequential --parallel-workers 2 \
--cmake-force-configure \
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
. install/setup.bash
```

* launch command
```bash
ros2 launch ocs2_anymal_models visualize.launch.py
```