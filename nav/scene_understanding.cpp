#include "scene_understanding.hpp"

#include <cmath>
#include <cstdio>

namespace nav
{

SceneUnderstanding::SceneUnderstanding(const SceneUnderstandingParams & p) : params_(p) {}

std::vector<Obstacle> SceneUnderstanding::process(const cv::Mat & bgr_image)
{
  std::vector<Obstacle> obstacles;
  detect_obstacles(bgr_image, obstacles);
  estimate_distances(obstacles);

  // Filter implausible detections
  obstacles.erase(
    std::remove_if(
      obstacles.begin(), obstacles.end(),
      [](const Obstacle & o) { return o.distance_m > 12.f || o.distance_m < 0.2f; }),
    obstacles.end());

  return obstacles;
}

void SceneUnderstanding::detect_obstacles(const cv::Mat & bgr, std::vector<Obstacle> & out)
{
  cv::Mat gray, edges;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
  cv::Canny(gray, edges, 60, 150);

  // Only consider the lower 60% of the image (below approximate horizon)
  int horizon = bgr.rows * 2 / 5;
  cv::Mat roi = edges(cv::Rect(0, horizon, edges.cols, edges.rows - horizon));

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & cnt : contours) {
    double area = cv::contourArea(cnt);
    if (area < 4000) continue;

    cv::Rect r = cv::boundingRect(cnt);
    // Shift back to full-image coordinates
    r.y += horizon;

    // Skip very wide boxes (wall-floor boundary)
    if (r.width > bgr.cols * 0.7f) continue;

    // Skip extreme aspect ratios
    float ar = static_cast<float>(r.width) / std::max(r.height, 1);
    if (ar > 3.5f || ar < 0.25f) continue;

    Obstacle o;
    o.box = cv::Rect2f(
      static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.width),
      static_cast<float>(r.height));
    o.label = "obstacle";
    o.is_moving = false;
    out.push_back(o);
  }

  // Aggressive NMS for overlapping boxes
  for (size_t i = 0; i < out.size(); ++i) {
    if (out[i].box.area() <= 0.f) continue;
    for (size_t j = i + 1; j < out.size(); ++j) {
      if (out[j].box.area() <= 0.f) continue;
      cv::Rect2f inter = out[i].box & out[j].box;
      if (inter.area() > 0.f) out[j].box = cv::Rect2f();
    }
  }
  out.erase(
    std::remove_if(out.begin(), out.end(), [](const Obstacle & o) { return o.box.area() <= 0.f; }),
    out.end());
}

void SceneUnderstanding::estimate_distances(std::vector<Obstacle> & obstacles)
{
  for (auto & o : obstacles) {
    float foot_y = o.box.y + o.box.height;
    float denom = foot_y - params_.cy;
    if (denom < 1.0f) denom = 1.0f;
    o.distance_m = params_.camera_height_m * params_.fy / denom;
  }
}

cv::Mat SceneUnderstanding::draw(const cv::Mat & image, const std::vector<Obstacle> & obs)
{
  cv::Mat output = image.clone();
  for (const auto & o : obs) {
    cv::Scalar color(0, 200, 255);  // yellow
    cv::rectangle(output, o.box, color, 2);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.1fm", static_cast<double>(o.distance_m));
    cv::putText(
      output, buf, cv::Point(static_cast<int>(o.box.x), static_cast<int>(o.box.y) - 6),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  }
  return output;
}

}  // namespace nav
