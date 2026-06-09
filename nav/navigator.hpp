#ifndef NAV__NAVIGATOR_HPP
#define NAV__NAVIGATOR_HPP

#include <string>
#include <vector>

#include "landmark_map.hpp"
#include "scene_understanding.hpp"

namespace nav
{

/// Navigation message from the system.
struct NavMessage
{
  enum Type { NONE, WARNING, GUIDANCE, ARRIVAL };
  Type type = NONE;
  std::string text;
};

/// Generates obstacle warnings and navigation guidance by combining
/// the current position (from VB-GPS), scene understanding, and the landmark map.
///
/// Implements the decision logic from the paper (Fig. 9):
///   1. Obstacle warnings have highest priority
///   2. If no obstacle: check proximity to landmarks → navigation guidance
class Navigator
{
public:
  explicit Navigator(const LandmarkMap & map);

  /// Process one frame and return the message to speak.
  NavMessage process(const Eigen::Vector3f & position, float yaw_deg,
                     const std::vector<Obstacle> & obstacles);

  // ---- configurable thresholds ----
  float static_warn_dist_m = 2.0f;   // warn for static obstacles within this range
  float moving_warn_dist_m = 4.0f;   // warn for moving obstacles (doubled for safety)
  float ready_dist_m = 2.0f;         // "ready to" message at this distance
  float action_dist_m = 1.0f;        // "action" message at this distance

private:
  const LandmarkMap & map_;
  const Landmark * active_landmark_ = nullptr; // currently tracked landmark
};

}  // namespace nav

#endif  // NAV__NAVIGATOR_HPP
