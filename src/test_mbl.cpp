#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "mbl.hpp"

int main(int argc, char ** argv)
{
  const std::string model_path = (argc >= 2) ? argv[1] : "assets/colmap_basement/model.bin";
  const std::string video_path = (argc >= 3) ? argv[2] : "assets/basement.mp4";

  nav::Mbl mbl(model_path);
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) { std::cerr << "Cannot open " << video_path << std::endl; return 1; }

  cv::Mat frame;
  int idx = 0;
  while (true) {
    cap >> frame;
    if (frame.empty()) break;
    ++idx;
    if (idx % 30 != 0) continue;

    auto r = mbl.localize(frame);
    if (r.valid)
      std::cout << "[F " << idx << "] inliers=" << r.inliers
                << " area=" << r.distribution_area
                << " t=[" << r.t_cw.x() << "," << r.t_cw.y() << "," << r.t_cw.z() << "]"
                << std::endl;
    else
      std::cout << "[F " << idx << "] FAILED" << std::endl;
  }
  return 0;
}
