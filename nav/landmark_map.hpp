#ifndef NAV__LANDMARK_MAP_HPP
#define NAV__LANDMARK_MAP_HPP

#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

namespace nav
{

/// A single annotated landmark in the 3D model, matching the paper's XML schema.
struct Landmark
{
  Eigen::Vector3f position;   // world coordinates (from SLAM map)
  float yaw_deg = 0.f;        // orientation at this point

  std::string on_left;        // "What is on the left-hand side?"
  std::string on_right;       // "What is on the right-hand side?"
  std::string caution;        // "What should you be most careful about?"
  std::string action;         // "What is the next step to the destination?"
};

/// In-memory landmark map.  Landmarks are pre-annotated (JSON file)
/// and queried by proximity during navigation.
class LandmarkMap
{
public:
  /// Load landmarks from a JSON file.
  bool load(const std::string & json_path);

  /// Find the nearest landmark within max_distance_m of the given position.
  /// Returns nullptr if none found.
  const Landmark * find_nearest(const Eigen::Vector3f & position, float max_distance_m = 2.0f) const;

  /// Number of loaded landmarks.
  size_t size() const { return landmarks_.size(); }

  const std::vector<Landmark> & landmarks() const { return landmarks_; }

private:
  std::vector<Landmark> landmarks_;
};

}  // namespace nav

#endif  // NAV__LANDMARK_MAP_HPP
