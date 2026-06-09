#include "landmark_map.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

namespace nav
{

bool LandmarkMap::load(const std::string & json_path)
{
  std::ifstream f(json_path);
  if (!f.is_open()) {
    std::cerr << "[LandmarkMap] Cannot open: " << json_path << std::endl;
    return false;
  }

  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception & e) {
    std::cerr << "[LandmarkMap] JSON parse error: " << e.what() << std::endl;
    return false;
  }

  landmarks_.clear();
  for (const auto & item : j) {
    Landmark lm;
    lm.position = Eigen::Vector3f(item.value("x", 0.f), item.value("y", 0.f),
                                  item.value("z", 0.f));
    lm.yaw_deg = item.value("yaw", 0.f);
    lm.on_left = item.value("on_left", "");
    lm.on_right = item.value("on_right", "");
    lm.caution = item.value("caution", "");
    lm.action = item.value("action", "");
    landmarks_.push_back(lm);
  }
  std::cout << "[LandmarkMap] Loaded " << landmarks_.size() << " landmarks." << std::endl;
  return true;
}

const Landmark * LandmarkMap::find_nearest(const Eigen::Vector3f & position,
                                            float max_distance_m) const
{
  const Landmark * best = nullptr;
  float best_dist = max_distance_m;
  for (const auto & lm : landmarks_) {
    float d = (lm.position - position).norm();
    if (d < best_dist) {
      best_dist = d;
      best = &lm;
    }
  }
  return best;
}

}  // namespace nav
