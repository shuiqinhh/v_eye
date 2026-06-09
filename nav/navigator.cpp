#include "navigator.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace nav
{

Navigator::Navigator(const LandmarkMap & map) : map_(map) {}

NavMessage Navigator::process(const Eigen::Vector3f & position, float yaw_deg,
                               const std::vector<Obstacle> & obstacles)
{
  NavMessage msg;

  // --- Priority 1: obstacle warnings ---
  for (const auto & o : obstacles) {
    float threshold = o.is_moving ? moving_warn_dist_m : static_warn_dist_m;
    if (o.distance_m > threshold) continue;

    // Determine relative direction
    float cx = o.box.x + o.box.width * 0.5f;
    // 1440px wide — left third / right third
    std::string dir;
    if (cx < 480) dir = "left";
    else if (cx > 960) dir = "right";
    else dir = "ahead";

    msg.type = NavMessage::WARNING;
    if (o.is_moving)
      msg.text = dir + ", " + o.label + " moving";
    else
      msg.text = dir + ", " + o.label;
    return msg; // highest priority — stop here
  }

  // --- Priority 2: navigation guidance via landmarks ---
  const Landmark * lm = map_.find_nearest(position, 5.0f);
  if (!lm) return msg; // no landmark nearby — stay silent

  float dist = (lm->position - position).norm();

  if (dist < action_dist_m) {
    // Check if this is the destination (last landmark)
    if (lm == &map_.landmarks().back()) {
      msg.type = NavMessage::ARRIVAL;
      msg.text = "arrived at destination. " + lm->caution;
    } else {
      msg.type = NavMessage::GUIDANCE;
      msg.text = lm->action;
    }
  } else if (dist < ready_dist_m) {
    msg.type = NavMessage::GUIDANCE;
    msg.text = "ready to " + lm->action;
  }

  return msg;
}

}  // namespace nav
