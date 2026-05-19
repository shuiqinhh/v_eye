#ifndef TOOLS__HIK_VIDEO_RECORDER_HPP
#define TOOLS__HIK_VIDEO_RECORDER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "io/camera.hpp"

namespace tools
{

class HikVideoRecorder
{
public:
  explicit HikVideoRecorder(const std::string & config_path);

  ~HikVideoRecorder();

  void start_recording();
  void stop_recording();
  bool is_recording() const;

private:
  void recording_loop();

  std::unique_ptr<io::CameraBase> camera_;

  std::thread recording_thread_;
  std::atomic<bool> recording_;
  std::atomic<bool> quit_;

  int fps_;
  std::string output_path_;
};

}  // namespace tools

#endif  // TOOLS__HIK_VIDEO_RECORDER_HPP
