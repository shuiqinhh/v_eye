#ifndef NAV__MBL_HPP
#define NAV__MBL_HPP

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

namespace nav
{

/// Result of one MBL localisation attempt.
struct MblResult
{
  bool valid = false;
  Eigen::Matrix3f R_cw;              // rotation world -> camera
  Eigen::Vector3f t_cw;              // translation world -> camera
  int inliers = 0;                   // number of RANSAC inliers
  float distribution_area = 0.f;     // area of fitted ellipse / image area
};

/// Simplified Model-Based Localisation.
/// Loads a pre-built 3D point cloud + ORB descriptors, then estimates
/// global camera pose from a single image via 2D-3D matching + PnP.
class Mbl
{
public:
  /// @param model_path  Binary model file from ORB-SLAM3 SaveModel()
  explicit Mbl(const std::string & model_path);

  /// Run localisation on a single image (typically a SLAM keyframe).
  MblResult localize(const cv::Mat & bgr_image);

private:
  void load_model(const std::string & path);

  // --- model data ---
  std::vector<cv::Point3f> points_3d_;    // 3D coordinates
  cv::Mat descriptors_;                   // ORB descriptors (N x 32, CV_8U)

  // --- camera intrinsics (from config) ---
  cv::Mat K_;

  // --- ORB feature detector / descriptor ---
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::BFMatcher> matcher_;
};

}  // namespace nav

#endif  // NAV__MBL_HPP
