#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include "System.h"
#include "SemanticMapper.h"
#include "VBGPSFusion.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./V_Eye path/to/vocabulary path/to/config" << std::endl;
        return 1;
    }

    // 1. 初始化 SLAM 系统
    V_Eye::System SLAM(argv[1], argv[2]);

    // 2. 初始化语义分割模块（使用已安装的库）
    V_Eye::SemanticMapper semanticMapper;

    // 3. 初始化融合定位模块
    V_Eye::VBGPSFusion fusion;

    // 4. 打开摄像头（索引 0 表示默认摄像头）
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }

    std::cout << "=== V-Eye 导航系统启动 ===" << std::endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        double timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // ---- 实时里程计 ----
        cv::Mat pose = SLAM.TrackMonocular(frame, timestamp);

        // ---- 语义分割 (异步，不阻塞主线程) ----
        std::thread semanticThread(&V_Eye::SemanticMapper::update, &semanticMapper, frame);

        // ---- 若跟踪丢失，调用重定位模块 ----
        if (SLAM.IsTrackingLost()) {
            cv::Mat globalPose = fusion.Relocalize(frame);
            if (!globalPose.empty()) {
                SLAM.ResetWithPose(globalPose);
                std::cout << "[系统] 重定位成功" << std::endl;
            }
        }

        // ---- 简单控制台输出 ---- 
        std::cout << "[" << timestamp << "] 当前位姿: " << pose.size() << std::endl;

        semanticThread.detach();   // 异步线程自动清理

        if (cv::waitKey(1) == 'q') break;   // 按 q 退出
    }

    cap.release();
    return 0;
}