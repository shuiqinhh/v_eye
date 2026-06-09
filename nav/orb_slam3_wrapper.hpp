#ifndef NAV__ORB_SLAM3_WRAPPER_HPP
#define NAV__ORB_SLAM3_WRAPPER_HPP

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <string>

namespace ORB_SLAM3
{
class System;
}

namespace nav
{

/// Lightweight wrapper around ORB-SLAM3::System for monocular tracking.
/// Returns relative poses and notifies keyframes for the VB-GPS pipeline.
class OrbSlam3Wrapper
{
public:
  /// @param vocab_path   Path to ORBvoc.txt
  /// @param settings_path Path to ORB-SLAM3 YAML settings file
  /// @param use_viewer    Enable Pangolin viewer (disable on headless)
  OrbSlam3Wrapper(
    const std::string & vocab_path, const std::string & settings_path, bool use_viewer);

  ~OrbSlam3Wrapper();

  // ---- per-frame API ----

  /// Feed one image to the SLAM system.
  /// @returns true if tracking is ongoing (not lost / not shut down).
  bool track(const cv::Mat & bgr_image, double timestamp_sec);

  /// Current camera pose (world -> camera, i.e. Tcw).
  Sophus::SE3f get_pose() const { return last_pose_; }

  /// Whether the last Track() call recovered from a lost state or re-initialised.
  bool is_relocalised() const;

  /// Whether ORB-SLAM3 is currently lost.
  bool is_lost() const;

  /// Signal all threads to finish and join them.
  void shut_down();

  // ---- key-frame / frame counter ----
  unsigned int keyframe_count() const { return keyframe_count_; }
  unsigned int frame_count() const { return frame_count_; }

  // ---- map export ----
  /// Number of map points in the current map.
  unsigned int map_point_count() const;
  /// Export all map points as a PLY file (for visualisation).
  void save_map_points(const std::string & path);
  /// Save 3D points + ORB descriptors as a binary model file (for MBL).
  void save_model(const std::string & path);

private:
  ORB_SLAM3::System * system_ = nullptr;
  Sophus::SE3f last_pose_;
  int last_state_ = -1;
  unsigned int keyframe_count_ = 0;
  unsigned int frame_count_ = 0;
};

}  // namespace nav

#endif  // NAV__ORB_SLAM3_WRAPPER_HPP
