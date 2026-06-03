#pragma once
#include <opencv2/opencv.hpp>

namespace V_Eye {

class SemanticMapper {
public:
    SemanticMapper();
    ~SemanticMapper();

    // 对输入图像进行语义分割，并更新内部语义地图
    void update(const cv::Mat& image);

    // 获取当前帧的语义掩码
    cv::Mat getSemanticMask();

private:
    struct Impl;
    Impl* pImpl;
};

} // namespace V_Eye