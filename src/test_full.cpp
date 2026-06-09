/// Full pipeline test: SLAM + MBL + VB-GPS + Scene Understanding + Navigation.
/// Usage: ./test_full [model.bin] [video] [landmarks.json]

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "landmark_map.hpp"
#include "mbl.hpp"
#include "navigator.hpp"
#include "orb_slam3_wrapper.hpp"
#include "scene_understanding.hpp"
#include "vb_gps_integrator.hpp"

int main(int argc, char ** argv)
{
  const std::string model_path = (argc >= 2) ? argv[1] : "assets/colmap_hall/model.bin";
  const std::string video_path = (argc >= 3) ? argv[2] : "assets/hall.mp4";
  const std::string landmark_path = (argc >= 4) ? argv[3] : "assets/landmarks_hall.json";

  // ---------- Init all modules ----------
  auto slam = nav::OrbSlam3Wrapper("assets/ORBvoc.txt", "configs/orb_slam3.yaml", false);
  auto mbl = nav::Mbl(model_path);
  auto integrator = nav::VbGpsIntegrator(&mbl);
  auto scene = nav::SceneUnderstanding();
  nav::LandmarkMap lm;
  if (!lm.load(landmark_path)) return 1;
  nav::Navigator navigator(lm);

  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) { std::cerr << "Cannot open: " << video_path << std::endl; return 1; }

  double fps = cap.get(cv::CAP_PROP_FPS);
  if (fps <= 0) fps = 30.0;
  double dt = 1.0 / fps;

  cv::Mat frame;
  double ts = 0.0;
  int fusions = 0, msgs = 0;
  std::vector<nav::Obstacle> last_obstacles;
  nav::NavMessage last_msg;

  while (true) {
    cap >> frame;
    if (frame.empty()) break;

    // 1. SLAM tracking
    slam.track(frame, ts);
    auto slam_pose = slam.get_pose();
    integrator.add_slam_pose(slam_pose, ts);

    // 2. MBL fusion every 30 frames
    if (slam.frame_count() % 30 == 0) {
      integrator.fuse_keyframe(frame, ts);
      ++fusions;
    }

    // 3. Scene + Nav update every 10 frames
    if (slam.frame_count() % 10 == 0) {
      last_obstacles = scene.process(frame);
      auto pos = integrator.current_pose().translation();
      last_msg = navigator.process(pos, 0.f, last_obstacles);

      if (last_msg.type != nav::NavMessage::NONE) {
        ++msgs;
        const char * type_str = (last_msg.type == nav::NavMessage::WARNING)   ? "WARN"
                                : (last_msg.type == nav::NavMessage::ARRIVAL) ? "ARRIVE"
                                                                              : "GUIDE";
        std::cout << "[" << slam.frame_count() << "] " << type_str << ": " << last_msg.text
                  << std::endl;
      }
    }

    // 4. Display every frame (with cached results + SLAM status)
    cv::Mat vis = nav::SceneUnderstanding::draw(frame, last_obstacles);

    // Overlay SLAM status bar
    auto p = slam.get_pose().translation();
    auto i = integrator.current_pose().translation();
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "SLAM:[%.2f %.2f %.2f]  INT:[%.2f %.2f %.2f]  KFs:%u  %s",
                  static_cast<double>(p.x()), static_cast<double>(p.y()),
                  static_cast<double>(p.z()), static_cast<double>(i.x()),
                  static_cast<double>(i.y()), static_cast<double>(i.z()),
                  slam.keyframe_count(), slam.is_lost() ? "LOST" : "OK");
    cv::putText(vis, buf, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 255), 2);

    // Overlay last navigation message
    if (!last_msg.text.empty()) {
      cv::Scalar msg_color = last_msg.type == nav::NavMessage::WARNING
                               ? cv::Scalar(0, 0, 255)
                               : cv::Scalar(0, 255, 0);
      cv::putText(vis, last_msg.text, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  msg_color, 2);
    }

    cv::imshow("V-Eye Full Pipeline", vis);
    if (cv::waitKey(1) == 27) break; // ESC to quit

    ts += dt;
  }

  slam.shut_down();
  std::cout << "Done. " << fusions << " fusions, " << msgs << " messages." << std::endl;
  return 0;
}
