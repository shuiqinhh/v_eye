#include "Tracking.h"
#include <iostream>

namespace V_Eye {

struct Tracking::Impl {
    bool isRunning = false;
};

Tracking::Tracking() {
    pImpl = new Impl();
    std::cout << "[Tracking] 跟踪模块已初始化" << std::endl;
}

Tracking::~Tracking() {
    delete pImpl;
}

void Tracking::Start() {
    pImpl->isRunning = true;
    std::cout << "[Tracking] 开始跟踪" << std::endl;
}

void Tracking::Stop() {
    pImpl->isRunning = false;
    std::cout << "[Tracking] 停止跟踪" << std::endl;
}

} // namespace V_Eye