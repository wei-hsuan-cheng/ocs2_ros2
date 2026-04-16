# OCS2 Anymal Loopshaping MPC

This package provided a perceptive mpc demo to allow Anymal_c robot to cross different terrains.

## 1. Build the package

```bash
cd ~/ros2_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
export MAKEFLAGS=-j2 && \
export NINJAFLAGS=-j2 && \
colcon build --symlink-install \
--packages-up-to ocs2_anymal_loopshaping_mpc \
--executor sequential --parallel-workers 2 \
--cmake-force-configure \
--cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
. install/setup.bash
```

## 2. Perceptive MPC demo

In this launch file, you can tried different terrains.

### 2.1 basic step

```bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py
```

![basic](../../../.images/perception_basic.png)

### 2.2 side gap

```bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=side_gap.png
```



![side](../../../.images/perception_side.png)

### 2.3 gaps

```bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=gaps.png terrain_scale:=1.0 forward_distance:=7.0
```

![gap](../../../.images/perception_gap.png)

### 2.4 hurdles

```bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=hurdles.png terrain_scale:=0.7 forward_distance:=7.0
```

![hurdles](../../../.images/perception_hurdles.png)

### 2.5 stepping stones

```bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py terrain_name:=stepping_stones.png terrain_scale:=1.0 forward_distance:=7.0
```

![stones](../../../.images/perception_stones.png)