#ifndef NAV__VB_GPS_INTEGRATOR_HPP
#define NAV__VB_GPS_INTEGRATOR_HPP

#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>
#include <deque>
#include <vector>

#include "mbl.hpp"

namespace nav
{

/// VB-GPS Integrator: fuses visual SLAM (relative) + MBL (global) poses.
/// Implements Algorithm 1 from the V-Eye paper.

/// Thresholds matching the paper's values.
struct VbGpsParams
{
  float dist_threshold_m = 1.2f;
  float angle_threshold_deg = 10.0f;
  int min_inliers = 40;
  float min_dist_area = 1.0f / 8.0f;
};

class VbGpsIntegrator
{
public:
  /// @param mbl        Pre-loaded MBL instance (shared, caller owns it).
  VbGpsIntegrator(Mbl * mbl, const VbGpsParams & p = VbGpsParams());

  /// Feed the SLAM relative pose for the current frame.
  /// The integrator accumulates the predicted global pose from relative motions.
  void add_slam_pose(const Sophus::SE3f & T_wc_relative, double timestamp);

  /// When a keyframe image is available, trigger MBL and fuse the result.
  /// @returns true if MBL result was accepted.
  bool fuse_keyframe(const cv::Mat & image, double timestamp);

  /// The current best-estimate global pose (after fusion).
  Sophus::SE3f current_pose() const;

  /// Full trajectory (for export / evaluation).
  const std::vector<Sophus::SE3f> & trajectory() const { return trajectory_; }

private:
  bool validate(const MblResult & r, const Sophus::SE3f & predicted);

  Mbl * mbl_;
  VbGpsParams params_;

  // Current prediction from SLAM accumulation
  Sophus::SE3f predicted_pose_;
  double last_kf_timestamp_ = 0.0;

  // Trajectory storage
  std::vector<Sophus::SE3f> trajectory_;

  // For scale adaptation: record the last integration
  bool has_first_integration_ = false;
  Sophus::SE3f last_mbl_pose_;
  Sophus::SE3f last_predicted_at_kf_;
};

}  // namespace nav

#endif  // NAV__VB_GPS_INTEGRATOR_HPP
