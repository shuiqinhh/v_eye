#include "hik_video_recorder.hpp"

#include <stdexcept>

#include "io/hikrobot/hikrobot.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace tools
{

HikVideoRecorder::HikVideoRecorder(const std::string & config_path)
: recording_(false), quit_(false)
{
  auto yaml = tools::load(config_path);

  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto rotation_angle = tools::read<int>(yaml, "rotation_angle");

  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<io::HikRobot>(exposure_ms, gain, vid_pid, rotation_angle);
  } else if (camera_name == "hikrobot_sn") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    auto serial_number = tools::read<std::string>(yaml, "serial_number");
    camera_ =
      std::make_unique<io::HikRobot>(exposure_ms, gain, vid_pid, serial_number, rotation_angle);
  } else {
    throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
  }

  output_path_ = tools::read<std::string>(yaml, "output_path");
  fps_ = tools::read_or<int>(yaml, "fps", 30);
}

HikVideoRecorder::~HikVideoRecorder()
{
  if (recording_) stop_recording();
}

void HikVideoRecorder::start_recording()
{
  if (recording_) {
    tools::logger()->warn("Already recording!");
    return;
  }

  quit_ = false;
  recording_thread_ = std::thread{&HikVideoRecorder::recording_loop, this};

  tools::logger()->info("Recording started: {} @ {} fps", output_path_, fps_);
}

void HikVideoRecorder::stop_recording()
{
  if (!recording_) return;

  quit_ = true;
  if (recording_thread_.joinable()) recording_thread_.join();

  tools::logger()->info("Recording stopped: {}", output_path_);
}

bool HikVideoRecorder::is_recording() const { return recording_; }

void HikVideoRecorder::recording_loop()
{
  recording_ = true;

  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;

  camera_->read(frame, timestamp);

  cv::VideoWriter writer;
  int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
  bool opened = writer.open(output_path_, fourcc, fps_, frame.size());

  if (!opened) {
    tools::logger()->error("Failed to open VideoWriter for: {}", output_path_);
    recording_ = false;
    return;
  }

  writer.write(frame);

  auto frame_interval = std::chrono::microseconds(1000000 / fps_);
  auto last_write_time = std::chrono::steady_clock::now();

  while (!quit_) {
    camera_->read(frame, timestamp);

    auto now = std::chrono::steady_clock::now();
    if (now - last_write_time >= frame_interval) {
      writer.write(frame);
      last_write_time = now;
    }
  }

  writer.release();
  recording_ = false;
}

}  // namespace tools
