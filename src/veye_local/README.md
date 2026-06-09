# V-Eye 单机轻量化导航

在单台电脑上完成：**预建图（test_SLAM.mp4）→ MBL 定位（test_MBL.mp4）→ VB-GPS 融合**。

## 目录结构

```
ros2_ws/
├── maps/default/              # 预建图输出（运行阶段1后生成）
│   ├── MapPoints.pcd          # 3D 点云
│   ├── MapDatabase.bin        # 3D 点 + ORB 描述子（MBL 用）
│   └── KeyFrameTrajectory.txt # 关键帧轨迹
├── src/
│   ├── ORB_SLAM3_ROS2/       # ORB-SLAM3 ROS2 封装（已扩展 keyframe / map 导出）
│   ├── veye_msgs/           # 自定义消息
│   └── veye_local/          # MBL、整合器、视频播放、launch
└── MapPoints.pcd              # 旧版默认输出（现已改为 maps/default/）
```

## 依赖

```bash
sudo apt install ros-humble-cv-bridge python3-opencv python3-yaml python3-scipy
pip3 install scipy  # 若 apt 版本不可用
```

## 编译

```bash
cd /home/rm/Desktop/ros2_ws
colcon build --symlink-install --packages-select veye_msgs veye_local orbslam3
source install/setup.bash
```

## 阶段 1：预建图（test_SLAM.mp4）

```bash
source /home/rm/Desktop/ros2_ws/install/setup.bash
ros2 launch veye_local offline_mapping.launch.py
```

视频播完后按 `Ctrl+C` 结束 SLAM 节点，地图写入 `maps/default/`。

也可手动分终端运行：

```bash
# 终端1
ros2 run veye_local video_player \
  /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/videos/test_SLAM.mp4 1.0 false

# 终端2
ros2 run orbslam3 mono \
  /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt \
  /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/config/monocular/test.yaml \
  --ros-args -p map_output_dir:=/home/rm/Desktop/ros2_ws/src/maps
```

## 阶段 2：在线定位 + MBL + 融合（test_MBL.mp4）

```bash
ros2 launch veye_local online_localization.launch.py
```

## 主要 Topic

| Topic | 说明 |
|-------|------|
| `/camera` | 视频帧输入 |
| `/orb_slam3/camera_pose` | SLAM 相对位姿 |
| `/orb_slam3/tracking_state` | 跟踪状态（2=OK, 4=LOST） |
| `/veye/keyframe` | 关键帧（约 1.5s，供 MBL） |
| `/veye/global_pose` | MBL 全局位姿 |
| `/veye/fused_pose` | VB-GPS 融合位姿 |
| `/veye/fused_pose_stamped` | 融合位姿（PoseStamped，可 RViz） |

## 参数

编辑 `config/default.yaml` 修改视频路径、地图目录、相机 yaml、MBL 阈值等。

Launch 覆盖示例：

```bash
ros2 launch veye_local offline_mapping.launch.py \
  video_path:=/path/to/map.mp4 map_dir:=/path/to/maps/site_a
```

## 模块说明

1. **ORB-SLAM3**：实时相对位姿，退出时导出点云与描述子数据库。
2. **mbl_localizer**：读取 `MapDatabase.bin`，ORB 匹配 + PnP RANSAC，输出全局位姿。
3. **integrator**：Sim(3) 对齐 SLAM 与 MBL，自适应尺度修正，输出平滑融合轨迹。

语义分割、地标地图、手机音频在本单机版中未实现，可在验证定位后再扩展。
