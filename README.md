# 2026传感与测试技术课程设计

***Reproducing the paper***： [***P. -J. Duh, Y. -C. Sung, L. -Y. F. Chiang, Y. -J. Chang and K. -W. Chen, "V-Eye: A Vision-Based Navigation System for the Visually Impaired," IEEE Transactions on Multimedia, vol. 23, pp. 1567-1580, 2021.***](https://ieeexplore.ieee.org/document/9113751)

---

## Quick Start

```bash
# 建图
./build/run_slam assets/basement.mp4 assets/colmap_basement

# 全流程测试（可视化）
./build/test_full assets/colmap_basement/model.bin assets/basement.mp4 assets/landmarks_basement.json
```

> ORBvoc.txt 和 .mp4 文件使用 Git LFS 管理，clone 后需用 `git lfs pull` 命令拉取。

---

## Prerequisites

### System
- Ubuntu 22.04+
- CMake >= 3.16
- GCC >= 9 (C++17，注意ORBSLAM-3需同步适配)

### Dependencies
```bash
# Essential
sudo apt install -y build-essential cmake pkg-config git
sudo apt install -y libopencv-dev libopencv-contrib-dev
sudo apt install -y libeigen3-dev libspdlog-dev libfmt-dev libyaml-cpp-dev nlohmann-json3-dev
sudo apt install -y libavcodec-dev libswscale-dev libavutil-dev libusb-1.0-0-dev
sudo apt install -y libboost-serialization-dev libssl-dev

# Pangolin (for ORB-SLAM3 viewer)
sudo apt install -y libglew-dev libepoxy-dev libgl1-mesa-dev

# Optional
sudo apt install -y cloudcompare
```

### Pangolin
```bash
git clone https://github.com/stevenlovegrove/Pangolin.git --depth 1
cd Pangolin
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build build -j$(nproc)
sudo cmake --build build --target install
sudo ldconfig
```

### ORB-SLAM3
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git --recursive --depth 1 /tmp/ORB_SLAM3

# Required patches:
# 1. /tmp/ORB_SLAM3/CMakeLists.txt: change C++11 check to set(CMAKE_CXX_STANDARD 17)
# 2. /tmp/ORB_SLAM3/include/LoopClosing.h line 226: bool mnFullBAIdx → int mnFullBAIdx

cd /tmp/ORB_SLAM3 && chmod +x build.sh && ./build.sh
```

### Git LFS
```bash
sudo apt install -y git-lfs
git lfs install
git lfs pull  # after clone
```

---

## Build

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Verify

```bash
# Test MBL global localization
./build/test_mbl

# Test scene understanding
./build/test_scene

# Test full pipeline (GUI required)
./build/test_full
```

---

## Targets

| Target | Description |
|--------|-------------|
| `run_slam` | Build 3D point cloud map from video |
| `test_mbl` | Test model-based localization (PnP) |
| `test_vb_gps` | Test SLAM + MBL fusion (VB-GPS) |
| `test_scene` | Test obstacle detection + distance |
| `test_full` | Full pipeline with visualization |
| `record_video` | Record video from HikRobot camera |
| `capture_withoutq` | Capture calibration images |
| `calibrate_camera` | Camera intrinsic calibration |

## Camera Setup

1. Connect HikRobot USB camera
2. Find VID/PID: `lsusb`
3. Update `configs/orb_slam3.yaml` with camera intrinsics

## Landmarks

Edit `assets/landmarks_basement.json` with actual coordinates from CloudCompare:

```bash
CloudCompare assets/colmap_basement/map.ply
# Click key locations → note x,y,z → update JSON
```

## License

- Original paper: IEEE TMM 2021
- Course project: Sensing and Testing Technology,School of Mechanical Engneering,Tongji University, 2026
- HikRobot SDK: (c) HikRobot
- ORB-SLAM3: GPLv3
