#pragma once
#include <opencv2/opencv.hpp>
#include <string>

namespace V_Eye {

class System {
public:
    System(const std::string& vocabPath, const std::string& configPath);
    ~System();

    // 跟踪单目图像帧，返回当前相机位姿
    cv::Mat TrackMonocular(const cv::Mat& image, double timestamp);

    // 检查跟踪状态是否丢失
    bool IsTrackingLost() const;

    // 用给定的位姿重置系统（供重定位调用）
    void ResetWithPose(const cv::Mat& pose);

    void Shutdown();

private:
    struct Impl;
    Impl* pImpl;
};

} // namespace V_Eye