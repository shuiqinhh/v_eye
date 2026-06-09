#ifndef NAV__SCENE_UNDERSTANDING_HPP
#define NAV__SCENE_UNDERSTANDING_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace nav
{

/// Detected obstacle with distance and classification.
struct Obstacle
{
  cv::Rect2f box;            // bounding box in image coordinates
  std::string label;         // "person", "object", etc.
  float distance_m = 0.f;    // estimated distance in meters
  bool is_moving = false;    // moving vs static
};

/// Lightweight scene understanding: obstacle detection + distance estimation.

struct SceneUnderstandingParams
{
  float camera_height_m = 1.2f;
  float fx = 2601.58f;
  float fy = 2585.13f;
  float cx = 778.77f;
  float cy = 378.09f;
};

class SceneUnderstanding
{
public:
  explicit SceneUnderstanding(const SceneUnderstandingParams & p = SceneUnderstandingParams());

  /// Process a new frame and return detected obstacles.
  std::vector<Obstacle> process(const cv::Mat & bgr_image);

private:
  void detect_obstacles(const cv::Mat & bgr, std::vector<Obstacle> & out);
  void estimate_distances(std::vector<Obstacle> & obstacles);

  SceneUnderstandingParams params_;

  // ---- colour helpers for visualisation ----
public:
  static cv::Mat draw(const cv::Mat & image, const std::vector<Obstacle> & obs);
};

}  // namespace nav

#endif  // NAV__SCENE_UNDERSTANDING_HPP
