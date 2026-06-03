#pragma once
#include <opencv2/opencv.hpp>

namespace V_Eye {

class Tracking {
public:
    Tracking();
    ~Tracking();

    void Start();
    void Stop();

private:
    struct Impl;
    Impl* pImpl;
};

} // namespace V_Eye