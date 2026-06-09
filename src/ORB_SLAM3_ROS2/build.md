# ORB_SLAM3 + ORB_SLAM3_ROS2 编译指南

> 测试环境：Ubuntu 22.04 (WSL2) · ROS2 Humble · OpenCV 4.x

---

## 一、编译 ORB_SLAM3

### 1.1 克隆仓库

```bash
cd ~
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
cd ORB_SLAM3
```

> 该 fork 修复了原版 ORB_SLAM3 在 Stereo 模式下的已知问题，推荐使用。  
> 原版地址：https://github.com/UZ-SLAMLab/ORB_SLAM3

### 1.2 安装依赖

```bash
# Eigen3（通常已预装）
sudo apt install libeigen3-dev

# Pangolin（可视化库）
sudo apt install libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev
git clone https://github.com/stevenlovegrove/Pangolin.git ~/Pangolin
cd ~/Pangolin && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install

# OpenCV（建议 4.x）
sudo apt install libopencv-dev
```

### 1.3 编译 ORB_SLAM3

```bash
cd ~/ORB_SLAM3
chmod +x build.sh
./build.sh
```

> `build.sh` 会依次编译 Thirdparty 中的 DBoW2、g2o、Sophus，再编译 ORB_SLAM3 本体。  
> 编译产物（`libORB_SLAM3.so`）位于 `~/ORB_SLAM3/lib/`。  
> **无需执行 `make install`**，后续 ROS2 包通过路径直接引用。

### 1.4 安装 Sophus（供 ROS2 包查找头文件）

```bash
cd ~/ORB_SLAM3/Thirdparty/Sophus/build
sudo make install
```

---

## 二、编译 ORB_SLAM3_ROS2

### 2.1 安装 ROS2 相关依赖

```bash
sudo apt install ros-humble-vision-opencv ros-humble-message-filters
```

### 2.2 克隆仓库到 ROS2 工作空间

```bash
mkdir -p ~/Desktop/ros2_ws/src
cd ~/Desktop/ros2_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git ORB_SLAM3_ROS2
```

### 2.3 配置 ORB_SLAM3 路径

编辑 `CMakeModules/FindORB_SLAM3.cmake`，将第 8 行改为你的 ORB_SLAM3 实际路径：

```cmake
set(ORB_SLAM3_ROOT_DIR "/home/rm/ORB_SLAM3")
```

### 2.4 配置 Python site-packages 路径

编辑 `CMakeLists.txt` 第 5 行，确认与当前 ROS2 发行版一致：

```cmake
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
```

### 2.5 编译

```bash
cd ~/Desktop/ros2_ws

# source ROS2 环境
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select orbslam3
```

---

## 三、运行

### 3.1 Source 工作空间

```bash
source ~/Desktop/ros2_ws/install/local_setup.bash
```

### 3.2 启动各模式

词典文件路径：`~/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt`  
配置文件路径：`~/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/config/`

```bash
# 单目
ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

# 双目
ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY

# RGB-D
ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

# 双目+IMU
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

---

## 四、常见问题

| 问题 | 解决方法 |
|------|---------|
| 找不到 `sophus/se3.hpp` | 执行 `cd ~/ORB_SLAM3/Thirdparty/Sophus/build && sudo make install` |
| 找不到 `libORB_SLAM3.so` | 检查 `FindORB_SLAM3.cmake` 中路径是否正确，确认 ORB_SLAM3 已编译 |
| OpenCV 版本冲突 | 建议统一使用 OpenCV 4.x，避免混用 3.x |
| `colcon build` 找不到 Pangolin | 确认已执行 `sudo make install` 安装 Pangolin |
