#include "SemanticMapper.h"
#include <iostream>

namespace V_Eye {

struct SemanticMapper::Impl {
    cv::Mat lastMask;

    cv::Mat segmentImage(const cv::Mat& image) {
        // 实际项目中可调用已安装的 YOLOv8 或 ONNX 模型
        // 这里用简单的模拟逻辑：生成随机颜色掩码
        cv::Mat mask(image.size(), CV_8UC3);
        cv::randu(mask, cv::Scalar(0,0,0), cv::Scalar(255,255,255));
        return mask;
    }
};

SemanticMapper::SemanticMapper() {
    pImpl = new Impl();
    std::cout << "[SemanticMapper] 语义分割模块已初始化" << std::endl;
}

SemanticMapper::~SemanticMapper() {
    delete pImpl;
}

void SemanticMapper::update(const cv::Mat& image) {
    if (image.empty()) return;
    pImpl->lastMask = pImpl->segmentImage(image);
}

cv::Mat SemanticMapper::getSemanticMask() {
    return pImpl->lastMask;
}

} // namespace V_Eye