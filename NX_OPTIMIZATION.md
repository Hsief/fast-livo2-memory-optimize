# FAST-LIVO2 Xavier NX 6GB bounded-memory profile

This branch is intended for **Livox Avia + 1920x1080 camera with camera scale=0.5**
on Jetson Xavier NX 6GB. The goal of v1 is to make runtime memory and latency
bounded before doing more aggressive algorithm changes.

## What was changed

### 1. Stop unbounded sensor backlog
- LiDAR ROS queue: 200000 -> configurable, default 4.
- Image ROS queue: 200000 -> configurable, default 3.
- IMU ROS queue: 200000 -> configurable, default 800.
- Internal LiDAR/Image/IMU buffers are bounded.
- Old data can be dropped instead of allowing real-time latency to grow without limit.

### 2. Remove large copies and allocator churn
- Livox CustomMsg is no longer deep-copied before preprocessing.
- Avia preprocessing runs outside the shared sensor-buffer mutex.
- 1920x1080 image is no longer deep-copied before resizing.
- With camera scale=0.5, the queued image is resized to the camera-model size before buffering.
- Point-cloud vectors retain capacity instead of swap/free/reallocate each frame.
- Voxel-map temporary buffers are reused.

### 3. Bound map memory
- Existing spatial map sliding remains enabled.
- Root voxel hash map reserves buckets up front.
- Emergency root-voxel hard cap deletes farthest root voxels in batches.
- Sparse visual map now has:
  - spatial local window,
  - max visual voxels,
  - max points per voxel,
  - max reference-frame age.
- Deleting old VisualPoints also releases their historical reference cv::Mat images.

### 4. Stop RViz from becoming part of the estimator
- Dense cloud publishing is disabled by default.
- Estimator cloud and visualization cloud are separated.
- RViz cloud gets an independent 0.25 m voxel filter.
- Large ROS publishers use queue size 1.
- nav_msgs/Path history is bounded and published less often.
- Serialization/image publication is skipped when there are no subscribers.
- Dedicated launch file starts with RViz disabled.

### 5. Stop hidden RAM growth from PCD export
The original Avia config used:
`pcd_save_en: true` + `interval: -1`.
That accumulates the entire map in RAM until shutdown.
The NX profile disables PCD/COLMAP online export by default.

## Build and run

```bash
cd ~/catkin_ws/src/fast-livo2-memory-optimize
git fetch origin
git checkout nx-bounded-memory-v1
git pull origin nx-bounded-memory-v1

cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

roslaunch fast_livo mapping_avia_nx.launch rviz:=false
```

For the first stability test, **do not run RViz on the NX**. Let FAST-LIVO2 run
for at least 10-20 minutes. If stable, test remote RViz from a PC. Only after
that test local RViz with:

```bash
roslaunch fast_livo mapping_avia_nx.launch rviz:=true
```

## Runtime telemetry to collect

FAST-LIVO2 prints once per second:

```text
[NX_MON] lag=... buffers(lidar=... img=... imu=... prop=...)
         root_voxels=... visual_voxels=... visual_points=...
         path=... pub_wait=... pcd_wait=...
         dropped(lidar=... img=... imu=...)
```

Timing lines:
```text
[NX_LIO] ...
[NX_VIO] ...
[NX_MAP] ...
[NX_VMAP] ...
```

Run the process-memory monitor in another terminal:

```bash
cd ~/catkin_ws/src/fast-livo2-memory-optimize
bash scripts/monitor_fast_livo_nx.sh | tee /tmp/fast_livo_mem.log
```

And Jetson telemetry:

```bash
tegrastats --interval 1000 | tee /tmp/tegrastats.log
```

## What to report after the first run

Please report:
1. Whether RViz was local, remote, or disabled.
2. Time until slowdown/freeze, or total stable runtime.
3. A few `[NX_MON]` lines from:
   - ~10 s,
   - ~1 min,
   - ~5 min,
   - just before slowdown/freeze.
4. `[NX_LIO]` and `[NX_VIO]` timing near the same moments.
5. Initial and final RSS from `fast_livo_mem.log`.
6. Any `[NX_MAP]` / `[NX_VMAP]` prune messages.
7. Peak RAM/SWAP and CPU/GPU information from `tegrastats`.

## Interpretation guide

- **RSS rises continuously while visual_points rises**:
  tighten visual-map bounds/reference age.
- **root_voxels rises continuously**:
  tighten spatial map or root-voxel cap.
- **lag rises while RSS stays flat**:
  estimator throughput is insufficient; optimize compute/downsampling next.
- **lag stays low but local RViz freezes**:
  visualization/GPU/shared-memory path is the bottleneck.
- **dropped_lidar rises quickly**:
  the estimator cannot consume Avia scans in real time; profile LIO stages.
- **pcd_wait must remain 0** for the normal online profile.
