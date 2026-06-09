#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "orb_slam3_wrapper.hpp"

int main(int argc, char ** argv)
{
  std::string input_path = "assets/hall.mp4";
  std::string out_dir = "assets/colmap_hall";
  if (argc >= 2) input_path = argv[1];
  if (argc >= 3) out_dir = argv[2];

  const std::string vocab_path = "assets/ORBvoc.txt";
  const std::string settings_path = "configs/orb_slam3.yaml";

  nav::OrbSlam3Wrapper slam(vocab_path, settings_path, false);

  cv::Mat frame;
  double timestamp = 0.0;

  bool is_dir = input_path.find('.') == std::string::npos; // crude: no extension = directory
  if (is_dir) {
    // --- Image directory mode ---
    std::vector<std::string> paths;
    for (const auto & e : std::filesystem::directory_iterator(input_path))
      if (e.path().extension() == ".jpg") paths.push_back(e.path().string());
    std::sort(paths.begin(), paths.end());
    std::cout << "Images: " << paths.size() << "  ->  " << out_dir << std::endl;
    for (const auto & p : paths) {
      frame = cv::imread(p);
      if (frame.empty()) continue;
      slam.track(frame, timestamp);
      timestamp += 1.0 / 30.0;
    }
  } else {
    // --- Video mode ---
    std::cout << "Video: " << input_path << "  ->  " << out_dir << std::endl;
    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) {
      std::cerr << "Cannot open video: " << input_path << std::endl;
      return 1;
    }
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;
    double dt = 1.0 / fps;
    while (true) {
      cap >> frame;
      if (frame.empty()) break;
      slam.track(frame, timestamp);
      timestamp += dt;
    }
  }

  std::string ply_path = out_dir + "/map.ply";
  std::string bin_path = out_dir + "/model.bin";
  std::string cmd = "mkdir -p " + out_dir;
  system(cmd.c_str());

  slam.save_map_points(ply_path);
  slam.save_model(bin_path);
  slam.shut_down();
  std::cout << "Done. " << slam.frame_count() << " frames, "
            << slam.keyframe_count() << " KFs -> " << out_dir << std::endl;
  return 0;
}
