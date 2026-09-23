# FAST-LIVO2/src 源码概览

这是无动态过滤的 FAST-LIVO2 基线源码。所有经过 deskew/downsample 的有效点都参与当前帧状态估计，并进入后续 voxel map 更新，因此它用于先验证传感器、时间戳、外参和基础建图链路。

## 运行入口

```bash
cd /home/rws/Desktop/CSM-LIDAR/step1_capture_mapping/map_ws
./run_fast_livo2.sh fast_livo rviz:=false
```

直接 launch 为 `roslaunch fast_livo mapping_avia.launch`。详细编译、输入输出和话题见 `map_ws/README.md`。

## 核心文件

```text
main.cpp              ROS node 和 LIVMapper 初始化
LIVMapper.cpp         LiDAR/IMU/相机同步、状态估计和地图更新
IMU_Processing.cpp    IMU 传播、去畸变和状态初始化
preprocess.cpp        Livox 点云预处理和时间排序
frame.cpp             图像帧/patch 数据
vio.cpp               稀疏直接法视觉更新
voxel_map.cpp         平面体素地图和点到平面匹配
visual_point.cpp      视觉地图点维护
```

## 数据边界

输入来自 Livox driver 和相机 workspace；输出通常在 `/fast_livo2/` namespace 下，包括 registered cloud、path、Laser_map 和 planes。该目录不判断行人/车辆是否动态，动态场景对基线地图产生的拖影正是其他四个变体要解决的问题。
