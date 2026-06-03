#include "VBGPSFusion.h"
#include <iostream>

namespace V_Eye {

struct VBGPSFusion::Impl {
    // 实际项目中可调用 DBoW2 词袋模型进行重定位
    cv::Mat computeGlobalPose(const cv::Mat& image) {
        // 模拟重定位：返回单位矩阵表示重定位成功
        return cv::Mat::eye(4, 4, CV_32F);
    }
};

VBGPSFusion::VBGPSFusion() {
    pImpl = new Impl();
    std::cout << "[VBGPSFusion] 融合定位模块已初始化" << std::endl;
}

VBGPSFusion::~VBGPSFusion() {
    delete pImpl;
}

cv::Mat VBGPSFusion::Relocalize(const cv::Mat& image) {
    if (image.empty()) return cv::Mat();
    return pImpl->computeGlobalPose(image);
}

} // namespace V_Eye