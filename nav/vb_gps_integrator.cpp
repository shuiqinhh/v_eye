#include "vb_gps_integrator.hpp"

#include <iostream>

namespace nav
{

VbGpsIntegrator::VbGpsIntegrator(Mbl * mbl, const VbGpsParams & p)
  : mbl_(mbl), params_(p), predicted_pose_()
{
}

void VbGpsIntegrator::add_slam_pose(const Sophus::SE3f & T_wc, double /*timestamp*/)
{
  predicted_pose_ = T_wc;
  trajectory_.push_back(T_wc);
}

bool VbGpsIntegrator::fuse_keyframe(const cv::Mat & image, double timestamp)
{
  if (!mbl_) return false;

  MblResult r = mbl_->localize(image);
  if (!r.valid) return false;

  Sophus::SE3f predicted = predicted_pose_;

  if (!validate(r, predicted)) return false;

  // ---------- accepted ----------
  Eigen::Matrix3f R_cw = r.R_cw;
  Sophus::SE3f T_mbl(R_cw, r.t_cw);

  if (has_first_integration_) {
    // Scale estimation between SLAM and MBL
    Eigen::Vector3f d_pred =
      predicted.translation() - last_predicted_at_kf_.translation();
    Eigen::Vector3f d_mbl = T_mbl.translation() - last_mbl_pose_.translation();
    float s = d_mbl.norm() / std::max(d_pred.norm(), 1e-6f);

    // Correct recent trajectory segment
    size_t seg_start = 0;
    if (trajectory_.size() > 30) seg_start = trajectory_.size() - 30;

    Eigen::Vector3f offset = T_mbl.translation() - s * predicted.translation();
    for (size_t i = seg_start; i < trajectory_.size(); ++i) {
      trajectory_[i].translation() = s * trajectory_[i].translation() + offset;
    }
  }

  trajectory_.push_back(T_mbl);
  predicted_pose_ = T_mbl;
  last_predicted_at_kf_ = predicted;
  last_mbl_pose_ = T_mbl;
  has_first_integration_ = true;
  last_kf_timestamp_ = timestamp;

  return true;
}

bool VbGpsIntegrator::validate(const MblResult & r, const Sophus::SE3f & predicted)
{
  if (r.distribution_area < params_.min_dist_area) return false;
  if (r.inliers < params_.min_inliers) return false;

  Eigen::Matrix3f R_cw = r.R_cw;
  Sophus::SE3f T_mbl(R_cw, r.t_cw);
  float dist = (T_mbl.translation() - predicted.translation()).norm();

  if (dist > params_.dist_threshold_m) {
    float angle =
      Eigen::AngleAxisf(T_mbl.rotationMatrix().transpose() * predicted.rotationMatrix())
        .angle();
    float angle_deg = angle * 180.0f / 3.14159f;
    if (angle_deg > params_.angle_threshold_deg) return false;
  }

  return true;
}

Sophus::SE3f VbGpsIntegrator::current_pose() const
{
  if (trajectory_.empty()) return Sophus::SE3f();
  return trajectory_.back();
}

}  // namespace nav
