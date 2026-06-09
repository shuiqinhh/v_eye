#include "orb_slam3_wrapper.hpp"

#include <Atlas.h>
#include <Map.h>
#include <System.h>

namespace nav
{

OrbSlam3Wrapper::OrbSlam3Wrapper(
  const std::string & vocab_path, const std::string & settings_path, bool use_viewer)
{
  system_ =
    new ORB_SLAM3::System(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, use_viewer);
}

OrbSlam3Wrapper::~OrbSlam3Wrapper()
{
  if (system_) {
    system_->Shutdown();
    delete system_;
    system_ = nullptr;
  }
}

bool OrbSlam3Wrapper::track(const cv::Mat & bgr_image, double timestamp_sec)
{
  if (!system_) return false;

  // TrackMonocular returns the current camera pose (Tcw).
  last_pose_ = system_->TrackMonocular(bgr_image, timestamp_sec);
  last_state_ = system_->GetTrackingState();
  ++frame_count_;

  // Detect new keyframes by counting.
  auto current_kf_count = system_->GetNumKeyFrames();
  if (current_kf_count > keyframe_count_) {
    keyframe_count_ = current_kf_count;
  }

  // Return true while tracking is active.
  // NOTE: do NOT call isFinished() — it calls GetTimeFromIMUInit() which
  // segfaults in pure-monocular (non-IMU) mode when LocalMapping isn't ready.
  return true;
}

bool OrbSlam3Wrapper::is_relocalised() const
{
  // Tracking state 2 = OK, 3 = recently lost. If we were lost and are now OK,
  // the system has relocalised. For simplicity we just check we're currently OK.
  return last_state_ == 2;
}

bool OrbSlam3Wrapper::is_lost() const { return system_ ? system_->isLost() : true; }

void OrbSlam3Wrapper::shut_down()
{
  if (system_) {
    system_->Shutdown();
    delete system_;
    system_ = nullptr;
  }
}

unsigned int OrbSlam3Wrapper::map_point_count() const
{
  return system_ ? system_->GetNumMapPoints() : 0;
}

void OrbSlam3Wrapper::save_map_points(const std::string & path)
{
  if (system_) system_->SaveMapPointsPLY(path);
}

void OrbSlam3Wrapper::save_model(const std::string & path)
{
  if (system_) system_->SaveModel(path);
}

}  // namespace nav
