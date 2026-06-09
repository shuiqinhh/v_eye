/// Test: run scene understanding on a video and show obstacle detections.

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "scene_understanding.hpp"

int main()
{
  const std::string video_path = "assets/basement.mp4";

  nav::SceneUnderstanding su;

  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cerr << "Cannot open " << video_path << std::endl;
    return 1;
  }

  cv::Mat frame;
  int count = 0;
  while (true) {
    cap >> frame;
    if (frame.empty()) break;
    ++count;

    // Process every 5th frame (HOG is slow)
    if (count % 5 != 0) continue;

    auto obstacles = su.process(frame);

    if (!obstacles.empty()) {
      std::cout << "[F " << count << "] " << obstacles.size() << " obstacles:";
      for (const auto & o : obstacles)
        std::cout << "  " << o.label << " @" << o.distance_m << "m"
                  << (o.is_moving ? " [MOVING]" : "");
      std::cout << std::endl;
    }

    cv::Mat vis = nav::SceneUnderstanding::draw(frame, obstacles);
    cv::imshow("Scene Understanding", vis);
    if (cv::waitKey(1) == 27) break;  // ESC to quit
  }

  return 0;
}
