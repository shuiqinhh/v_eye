#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "tools/hik_video_recorder.hpp"
#include "tools/logger.hpp"

namespace
{
volatile std::sig_atomic_t g_signal_flag = 0;
}

void signal_handler(int signum) { g_signal_flag = signum; }

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
    return 1;
  }

  std::string config_path = argv[1];

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  tools::HikVideoRecorder recorder(config_path);
  recorder.start_recording();

  tools::logger()->info("Press Ctrl+C to stop recording...");

  while (!g_signal_flag) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  tools::logger()->info("Stopping...");
  recorder.stop_recording();
  tools::logger()->info("Done.");

  return 0;
}
