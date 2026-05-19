# V-Eye: A Vision-Based Navigation System for the Visually Impaired

复现 IEEE TMM 2021 论文 **V-Eye** 的课程项目。利用海康机器人（HikRobot）工业相机、OpenCV 以及 ORB-SLAM3 算法，构建一套视觉辅助导航系统。

> 当前进度：已完成相机驱动、标定工具链和视频录制模块，导航模块（`nav/`）尚未实现。

---

## 目录结构

```
v_eye/
├── CMakeLists.txt              # 顶层 CMake 构建文件
├── README.md                   # 项目简介
├── .clang-format               # 代码格式化规则（Google Style）
├── V-Eye_*.pdf                 # 原始论文
│
├── configs/                    # 配置文件
│   ├── params.yaml             # 主配置（相机参数、标定结果、ORB-SLAM3 等参数）
│
├── io/                         # 相机 I/O 模块（静态库）
│   ├── camera.hpp/cpp          # 相机抽象层：根据 YAML 配置创建对应驱动
│   └── hikrobot/               # 海康机器人相机驱动
│       ├── hikrobot.hpp/cpp    # 驱动实现（帧抓取、USB 自动恢复、Bayer 转 RGB）
│       ├── include/            # 厂商 SDK 头文件（MvCameraControl.h 等）
│       └── lib/                # 厂商 SDK 动态库（amd64 / arm64）
│
├── tools/                      # 工具库（Object 库，编译进使用它的目标）
│   ├── hik_video_recorder.hpp/cpp  # 视频录制工具类
│   ├── thread_safe_queue.hpp   # 线程安全队列（模板类）
│   ├── yaml.hpp                # YAML 读写辅助（load / read / read_or）
│   ├── math_tools.hpp/cpp      # 数学工具（欧拉角、四元数、坐标转换等）
│   ├── img_tools.hpp/cpp       # 图像绘制工具（画点、连线、文字）
│   ├── plotter.hpp/cpp         # UDP 绘图器（将数据发到本地绘图服务器）
│   ├── logger.hpp/cpp          # 日志系统（基于 spdlog，同时输出到文件和终端）
│   └── exiter.hpp/cpp          # Ctrl+C 信号捕获器
│
├── src/                        # 用户可执行的入口程序
│   └── record_video.cpp        # 录制视频主程序
│
├── calibrate/                  # 标定程序
│   ├── capture_withoutq.cpp    # 图像采集（不需要 IMU / 四元数）
│   ├── calibrate_camera.cpp    # 相机内参标定（输出 ORB-SLAM3 格式 YAML）
│   ├── calibrate_device2world_handeye.cpp  # 手眼标定 + 世界坐标系标定
│   └── capture.cpp             # （已废弃）旧版采集，依赖不存在的 gimbal 模块
│
├── assets/                     # 资源文件
│   ├── basement_demo.mp4       # 论文 basement 场景演示视频
│   ├── img_from_demo/          # 从数据demo中抽取的帧
│   └── img_with_q/             # 采集的用于相机内参标定的图片
│
├── nav/                        # 导航模块（待实现）
├── build/                      # CMake 构建产出
└── logs/                       # 项目运行日志
```

---

## 各程序详解

### 1. 视频录制 — `record_video`

**源文件**：`src/record_video.cpp` + `tools/hik_video_recorder.cpp`

**功能**：连接海康相机，按指定帧率录制 MP4 视频。

**用法**：
```bash
./build/record_video configs/params.yaml
```
按 `Ctrl+C` 停止录制，视频自动保存。


**核心逻辑**：
- `HikVideoRecorder` 构造函数解析 YAML，根据 `camera_name` 创建 `HikRobot` 相机实例
- `start_recording()` 启动独立线程，循环读取相机帧并通过 OpenCV `VideoWriter` 写入 MP4 文件（mp4v 编码）
- 按设定的 FPS 控制写入间隔，避免帧过快
- 析构或 `stop_recording()` 释放资源

---

### 2. 图像采集 — `capture_withoutq`

**源文件**：`calibrate/capture_withoutq.cpp`

**功能**：实时预览相机画面，检测标定板（10×7 非对称圆点网格），按 `S` 键保存图像。用于相机内参标定的数据采集。

**用法**：
```bash
./build/capture_withoutq configs/params.yaml
```

**运行时按键**：
| 按键 | 功能 |
|------|------|
| `s`  | 保存当前帧（含检测到的角点位置）|
| `Esc`   `q` | 退出程序 |

**依赖**：需要 `configs/params.yaml` 中包含 `pattern_cols`、`pattern_rows` 等标定板参数。

---

### 3. 相机内参标定 — `calibrate_camera`

**源文件**：`calibrate/calibrate_camera.cpp`

**功能**：读取一组标定板图像，检测角点，调用 OpenCV `calibrateCamera` 计算相机内参矩阵和畸变系数，输出 **ORB-SLAM3 兼容**的 YAML 配置文件。

**用法**：
```bash
./build/calibrate_camera configs/params.yaml
```

**输入**：由 `capture_withoutq` 采集的标定图像。

**输出**：更新 `configs/params.yaml`，写入：
- `Camera.fx / fy / cx / cy`：内参矩阵
- `Camera.k1 / k2 / p1 / p2 / k3`：畸变系数
- `Camera.width / height / fps`：图像尺寸和帧率
- `ORBextractor.*`：ORB 特征提取参数
- `Viewer.*`：SLAM 可视化参数

**细节**：使用 `CALIB_FIX_K3` 固定 k3=0（适合小视场相机）。

---

### 5. 工具库 — `tools/`

这些类和函数被其他模块广泛调用：

| 文件 | 功能 |
|------|------|
| `yaml.hpp` | YAML 文件读取：`load(path)` 加载，`read<T>(key)` 读取必填字段，`read_or<T>(key, default)` 读取可选字段 |
| `logger.hpp/cpp` | 全局日志器（spdlog），自动创建 `logs/` 目录，同时输出到彩色终端和滚动日志文件 |
| `thread_safe_queue.hpp` | 模板类，阻塞式线程安全队列，支持容量限制和满队列回调 |
| `math_tools.hpp/cpp` | 欧拉角/四元数/旋转矩阵互转、球坐标转换、角度归一化、雅可比矩阵 |
| `img_tools.hpp/cpp` | 在 `cv::Mat` 上画点、连线、文字 |
| `plotter.hpp/cpp` | 通过 UDP 将 JSON 数据发送到 `127.0.0.1:9870` 的绘图服务器，方便实时可视化 |
| `exiter.hpp/cpp` | 单例，监听 SIGINT 信号，全局查询是否需要退出 |

---

### 6. 相机驱动 — `io/`

**`CameraBase`**（`io/camera.hpp`）：抽象接口，定义 `read(img, timestamp)` 纯虚函数。

**`Camera`**（`io/camera.hpp:cpp`）：工厂类，读取 YAML 中的 `camera_name` 字段，自动创建对应的相机驱动实例。当前支持：
- `hikrobot`：按 USB VID/PID 识别海康相机
- `hikrobot_sn`：按相机序列号识别（多相机场景）

**`HikRobot`**（`io/hikrobot/hikrobot.cpp`）：海康相机驱动核心实现：
- 使用海康 MvCameraControl SDK 进行设备枚举、连接、取流
- 自动将 Bayer 格式转换为 BGR 彩色图像（支持 GR/RG/GB/BG 四种 Bayer 模式）
- **守护线程**：每 100ms 检查取流线程是否存活，若断流则自动复位 USB 设备并重连
- 通过 `ThreadSafeQueue<1>` 缓冲最新一帧，读取端阻塞等待

---

## 环境搭建

### 系统要求
- Ubuntu 22.04+（或其他 Linux 发行版）
- CMake ≥ 3.16
- GCC ≥ 9（支持 C++17）
- x86_64（AMD64）或 ARM64 架构

### 依赖安装

```bash
# 基础编译工具
sudo apt install build-essential cmake pkg-config

# OpenCV（带 contrib 模块，用于标定板检测）
sudo apt install libopencv-dev libopencv-contrib-dev

# Eigen3（线性代数）
sudo apt install libeigen3-dev

# 日志与序列化
sudo apt install libspdlog-dev libfmt-dev
sudo apt install libyaml-cpp-dev nlohmann-json3-dev

# FFmpeg（视频编码）
sudo apt install libavcodec-dev libswscale-dev libavutil-dev

# libusb（USB 设备复位）
sudo apt install libusb-1.0-0-dev

# 可选：Intel OpenVINO（深度学习推理）
# 从 https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html 下载
```

### 连接海康相机

1. 将海康 USB 相机连接到电脑
2. 查看相机的 VID/PID：
   ```bash
   lsusb
   # 输出示例：Bus 001 Device 003: ID 2bdf:0001 ...
   ```
3. 在 `configs/params.yaml` 中填入正确的 `vid_pid`（格式 `XXXX:YYYY`）

### 编译

```bash
cd v_eye
cmake -B build -S .
cmake --build build -j$(nproc)
```

编译产物位于 `build/` 目录：
- `build/record_video`
- `build/capture_withoutq`
- `build/calibrate_camera`
- `build/calibrate_device2world_handeye`

---

## 使用流程

### 第一步：标定相机内参

1. 准备好标定板（10×7 非对称圆点网格，圆心距 40mm）
2. 修改 `configs/params.yaml` 中的标定板参数和相机参数
3. 采集标定图像：
   ```bash
   ./build/capture_withoutq configs/params.yaml
   ```
   将标定板放在不同位置和角度，按 `S` 保存图像（建议 20-30 张）
4. 运行内参标定：
   ```bash
   ./build/calibrate_camera configs/params.yaml
   ```
   标定结果自动写入 `configs/params.yaml`
   ```

### 第二步：录制视频

```bash
# 编辑配置
vim configs/video_record.yaml

# 开始录制
./build/record_video configs/video_record.yaml
# 按 Ctrl+C 停止
```

---

## 配置文件参考

`configs/params.yaml` 中的关键字段说明：

```yaml
# ──── 标定板参数 ────
pattern_cols: 10             # 圆点网格列数
pattern_rows: 7              # 圆点网格行数
center_distance_mm: 40       # 相邻圆心距（毫米）

# ──── 相机参数 ────
camera_name: "hikrobot"      # hikrobot / hikrobot_sn
exposure_ms: 11              # 曝光时间
gain: 10.0                   # 增益
vid_pid: "2bdf:0001"         # USB VID:PID
rotation_angle: 0            # 旋转角度（0/90/180）

# ──── 相机内参（由标定程序自动填写）───
Camera.fx: 2737.48
Camera.fy: 2731.61
Camera.cx: 644.90
Camera.cy: 352.11
Camera.k1: -0.1445
Camera.k2: 0.0922
Camera.p1: 0.0013
Camera.p2: 0.0009
Camera.k3: 0
Camera.width: 1440
Camera.height: 1080
Camera.fps: 30

# ──── ORB-SLAM3 参数 ────
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

---


## 许可证与参考

- 原始论文：[V-Eye: A Vision-Based Navigation System for the Visually Impaired](https://ieeexplore.ieee.org/document/9452156)
- 本项目为课程项目（Sensing and Testing Technology, 2026）
- 海康机器人 SDK 版权归杭州海康机器人技术有限公司所有
