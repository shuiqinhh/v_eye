# V-Eye: A Vision-Based Navigation System for the Visually Impaired

复现 IEEE TMM 2021 论文 **V-Eye** 的课程项目。利用海康机器人（HikRobot）工业相机、OpenCV、ORB-SLAM3 以及自定义 MBL/VB-GPS 模块，构建一套完整的视觉辅助导航系统。

> 当前进度：**全部模块已完成** —— 相机驱动、标定、SLAM 建图、MBL 全局定位、VB-GPS 融合、场景理解和导航消息生成。

---

## 目录结构

```
v_eye/
├── CMakeLists.txt              # 顶层 CMake 构建文件
├── README.md                   # 项目简介 + 部署指南
├── AGENTS.md                   # 本文件：开发者文档
├── .clang-format               # 代码格式化规则
│
├── configs/                    # 配置文件
│   ├── params.yaml             # 标定 + 相机参数
│   └── orb_slam3.yaml          # ORB-SLAM3 专用配置
│
├── io/                         # 相机 I/O 模块（静态库）
│   ├── camera.hpp/cpp          # 相机工厂类
│   └── hikrobot/               # 海康机器人驱动
│       ├── hikrobot.hpp/cpp    # 帧抓取、USB 恢复、Bayer→RGB
│       ├── include/            # 厂商 SDK 头文件
│       └── lib/                # 厂商 SDK 动态库
│
├── tools/                      # 工具库（Object 库）
│   ├── math_tools.hpp/cpp      # 欧拉角/四元数/坐标变换
│   ├── img_tools.hpp/cpp       # 图像绘制
│   ├── plotter.hpp/cpp         # UDP 数据可视化
│   ├── logger.hpp/cpp          # 日志（spdlog，终端+文件）
│   ├── exiter.hpp/cpp          # SIGINT 信号捕获
│   ├── thread_safe_queue.hpp   # 线程安全队列
│   ├── hik_video_recorder.hpp/cpp  # MP4 录制
│   └── yaml.hpp                # YAML 读写
│
├── nav/                        # 导航核心模块（静态库）
│   ├── orb_slam3_wrapper.hpp/cpp     # ORB-SLAM3 封装
│   ├── mbl.hpp/cpp                   # 全局定位（2D-3D PnP）
│   ├── vb_gps_integrator.hpp/cpp     # SLAM + MBL 融合
│   ├── scene_understanding.hpp/cpp   # 障碍物检测 + 测距
│   ├── landmark_map.hpp/cpp          # 地标地图加载/查询
│   └── navigator.hpp/cpp             # 导航消息生成
│
├── src/                        # 可执行程序入口
│   ├── record_video.cpp        # 录制视频
│   ├── run_slam.cpp            # 建图（视频/图像序列）
│   ├── test_mbl.cpp            # 测试 MBL 全局定位
│   ├── test_vb_gps.cpp         # 测试 VB-GPS 融合定位
│   ├── test_scene.cpp          # 测试场景理解
│   └── test_full.cpp           # 全流程可视化测试
│
├── calibrate/                  # 标定程序
│   ├── capture_withoutq.cpp    # 标定图像采集
│   ├── calibrate_camera.cpp    # 相机内参标定
│   └── calibrate_device2world_handeye.cpp  # 手眼标定
│
├── assets/                     # 资源文件
│   ├── hall.mp4                # hall 场景视频
│   ├── basement.mp4            # basement 场景视频
│   ├── ORBvoc.txt              # ORB 词袋（139MB，Git LFS）
│   ├── colmap_hall/            # hall 场景的 3D 模型
│   │   ├── map.ply             #   点云（可视化）
│   │   └── model.bin           #   模型（MBL 用）
│   ├── colmap_basement/        # basement 场景的 3D 模型
│   │   ├── map.ply
│   │   └── model.bin
│   └── landmarks_basement.json # 地标标注
│
├── build/                      # CMake 构建产出
└── logs/                       # 运行日志
```

---

## 系统架构

```
┌──────────────┐     ┌─────────────────────┐     ┌──────────────────────┐
│ HikRobot相机 │ ──► │  ORB-SLAM3           │ ──► │  VB-GPS Integrator   │
│ (io/)        │     │  相对位姿 + 关键帧    │     │  SLAM + MBL 融合     │
└──────────────┘     └──────────┬───────────┘     └───────────┬──────────┘
                                │                             │
                                │ 关键帧                       │ 最终位姿
                                ▼                             ▼
                     ┌─────────────────────┐    ┌──────────────────────┐
                     │  MBL 全局定位        │    │  Navigator            │
                     │  2D ORB → 3D模型    │    │  地标查询 + 消息生成   │
                     │  PnP + RANSAC       │    └──────────┬───────────┘
                     └─────────────────────┘               │
                                                           ▼
                     ┌─────────────────────┐    ┌──────────────────────┐
                     │  SceneUnderstanding  │    │  输出                 │
                     │  边缘检测 + 单目测距 │    │  方向 + 距离 + 动作   │
                     └─────────────────────┘    └──────────────────────┘
```

---

## 各程序详解

### 建图 — `run_slam`

**用法**：
```bash
# 视频模式
./build/run_slam assets/basement.mp4 assets/colmap_basement

# 图像序列模式（目录路径不带扩展名）
./build/run_slam /path/to/images assets/colmap_basement
```

**输出**：`map.ply`（点云可视化）+ `model.bin`（MBL 模型文件）

---

### MBL 全局定位 — `test_mbl`

测试 2D-3D 特征匹配定位。

**用法**：
```bash
./build/test_mbl [model.bin] [video.mp4]
# 默认: assets/colmap_basement/model.bin assets/basement.mp4
```

**输出**：每 30 帧打印 inlier 数、特征分布面积、全局位姿。inlier >= 40 且分布面积 >= 1/8 为可信定位。

---

### VB-GPS 融合定位 — `test_vb_gps`

同时运行 SLAM + MBL，融合两者位姿。

**用法**：
```bash
./build/test_vb_gps [model.bin] [video.mp4]
```

**输出**：每 30 帧打印融合结果（Y/N）和融合后位姿。

---

### 场景理解 — `test_scene`

障碍物检测 + 距离估计。

**用法**：
```bash
./build/test_scene
```

**输出**：窗口显示黄色框标注的障碍物及距离。按 ESC 退出。

---

### 全流程 — `test_full`

**运行所有模块**，可视化窗口显示 SLAM 状态、障碍物、导航消息。

**用法**：
```bash
./build/test_full [model.bin] [video.mp4] [landmarks.json]
# 默认: basement 模型 + basement 视频 + landmarks_basement.json
```

**窗口信息**：
- 白色：SLAM 位姿 + 融合位姿 + 关键帧数 + 跟踪状态
- 黄色框：障碍物 + 距离
- 绿色字：导航引导
- 红色字：障碍告警

按 ESC 退出。

---

### 标定程序

| 程序 | 功能 |
|------|------|
| `capture_withoutq` | 实时预览，按 S 保存标定图像 |
| `calibrate_camera` | 内参标定，输出 ORB-SLAM3 格式 |
| `calibrate_device2world_handeye` | 手眼标定 + 世界坐标系标定 |

---

## MBL 模型文件格式

`model.bin` 为二进制格式：

```
[uint32_t] 点数 N
重复 N 次:
  [float ×3]  x, y, z（3D 坐标）
  [uint8 ×32] ORB 描述子（256-bit）
```

---

## 环境搭建

### 系统要求
- Ubuntu 22.04+
- CMake ≥ 3.16
- GCC ≥ 9（C++17）
- 8GB+ RAM（ORBvoc.txt 加载需 ~1.5GB）

### 依赖安装

```bash
# 基础工具
sudo apt install -y build-essential cmake pkg-config git

# OpenCV
sudo apt install -y libopencv-dev libopencv-contrib-dev

# Eigen3
sudo apt install -y libeigen3-dev

# 日志与序列化
sudo apt install -y libspdlog-dev libfmt-dev libyaml-cpp-dev nlohmann-json3-dev

# FFmpeg
sudo apt install -y libavcodec-dev libswscale-dev libavutil-dev

# USB
sudo apt install -y libusb-1.0-0-dev

# Pangolin（ORB-SLAM3 可视化）
sudo apt install -y libglew-dev libepoxy-dev libgl1-mesa-dev

# ORB-SLAM3 额外依赖
sudo apt install -y libboost-serialization-dev libssl-dev

# 可选
sudo apt install -y cloudcompare  # 点云可视化
```

### 编译

```bash
# 1. 编译 Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git --depth 1
cd Pangolin && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build build -j$(nproc) && sudo cmake --build build --target install
sudo ldconfig

# 2. 编译 ORB-SLAM3（需修改 CMakeLists.txt 为 C++17）
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git --recursive --depth 1 /tmp/ORB_SLAM3
# 编辑 /tmp/ORB_SLAM3/CMakeLists.txt：将 C++11 检查替换为 set(CMAKE_CXX_STANDARD 17)
# 编辑 /tmp/ORB_SLAM3/include/LoopClosing.h：将 bool mnFullBAIdx 改为 int
cd /tmp/ORB_SLAM3 && chmod +x build.sh && ./build.sh

# 3. 编译 v_eye
cd v_eye
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### 编译产出

| 程序 | 说明 |
|------|------|
| `build/run_slam` | ORB-SLAM3 建图 |
| `build/test_mbl` | MBL 全局定位测试 |
| `build/test_vb_gps` | VB-GPS 融合测试 |
| `build/test_scene` | 场景理解测试 |
| `build/test_full` | 全流程可视化测试 |
| `build/record_video` | 录制视频 |
| `build/capture_withoutq` | 标定图像采集 |
| `build/calibrate_camera` | 相机内参标定 |

---

## 配置文件参考

`configs/orb_slam3.yaml` 关键字段：

```yaml
Camera1.fx: 2601.58        # 内参
Camera1.fy: 2585.13
Camera1.cx: 778.77
Camera1.cy: 378.09
Camera1.k1/k2/p1/p2/k3     # 畸变系数
ORBextractor.nFeatures: 2000  # ORB 特征数
ORBextractor.iniThFAST: 20   # FAST 阈值
```

---

## 许可证与参考

- 原始论文：[V-Eye: A Vision-Based Navigation System for the Visually Impaired](https://ieeexplore.ieee.org/document/9113751)
- 本项目为课程项目（Sensing and Testing Technology,School of Mechanical Engneering,Tongji University, 2026）
- 海康机器人 SDK 版权归杭州海康机器人技术有限公司所有
- ORB-SLAM3 采用 GPLv3 许可证
