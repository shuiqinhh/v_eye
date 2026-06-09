#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "mbl.hpp"
#include "orb_slam3_wrapper.hpp"
#include "vb_gps_integrator.hpp"

int main(int argc, char ** argv)
{
  const std::string model_path = (argc >= 2) ? argv[1] : "assets/colmap_basement/model.bin";
  const std::string video_path = (argc >= 3) ? argv[2] : "assets/basement.mp4";

  nav::OrbSlam3Wrapper slam("assets/ORBvoc.txt", "configs/orb_slam3.yaml", false);
  nav::Mbl mbl(model_path);
  nav::VbGpsIntegrator integrator(&mbl);

  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) { std::cerr << "Cannot open " << video_path << std::endl; return 1; }

  double fps = cap.get(cv::CAP_PROP_FPS);
  if (fps <= 0) fps = 30.0;
  double dt = 1.0 / fps;
  cv::Mat frame;
  double ts = 0.0;
  int fusions = 0;

  while (true) {
    cap >> frame;
    if (frame.empty()) break;
    if (!slam.track(frame, ts)) break;
    integrator.add_slam_pose(slam.get_pose(), ts);

    if (slam.frame_count() % 30 == 0) {
      bool ok = integrator.fuse_keyframe(frame, ts);
      if (ok) ++fusions;
      auto t = integrator.current_pose().translation();
      std::cout << "[F " << slam.frame_count() << "] fused=" << (ok ? "Y" : "N")
                << " t=[" << t.x() << "," << t.y() << "," << t.z() << "]" << std::endl;
    }
    ts += dt;
  }
  slam.shut_down();
  std::cout << "Done. " << fusions << " fusions." << std::endl;
  return 0;
}
