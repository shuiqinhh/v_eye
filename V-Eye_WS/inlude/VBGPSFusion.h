#pragma once
#include <opencv2/opencv.hpp>

namespace V_Eye {

class VBGPSFusion {
public:
    VBGPSFusion();
    ~VBGPSFusion();

    // 全局重定位：输入图像，输出估计的全局位姿矩阵
    cv::Mat Relocalize(const cv::Mat& image);

private:
    struct Impl;
    Impl* pImpl;
};

} // namespace V_Eye