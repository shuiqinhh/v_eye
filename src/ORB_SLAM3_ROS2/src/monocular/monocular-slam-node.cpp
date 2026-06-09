#include "monocular-slam-node.hpp"

#include "map_database.hpp"
#include "map_point_cloud.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using std::placeholders::_1;

namespace
{

geometry_msgs::msg::Pose Se3ToPose(const Sophus::SE3f& Twc)
{
    geometry_msgs::msg::Pose pose;
    const auto t = Twc.translation();
    const auto q = Twc.unit_quaternion();
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

}  // namespace

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   rclcpp::Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    m_map_output_dir = this->declare_parameter<std::string>(
        "map_output_dir", "/home/rm/Desktop/ros2_ws/src/maps");
    m_keyframe_interval_sec = this->declare_parameter<double>("keyframe_interval_sec", 1.5);
    m_keyframe_min_translation = this->declare_parameter<double>("keyframe_min_translation", 0.10);
    m_keyframe_min_rotation_rad = this->declare_parameter<double>(
        "keyframe_min_rotation_deg", 5.0) * M_PI / 180.0;

    m_has_last_keyframe_pose = false;
    m_last_keyframe_time_sec = 0.0;
    m_last_keyframe_emit_sec = -1e9;
    m_map_saved = false;
    m_best_map_point_count = 0;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    m_mapping_done_subscriber = this->create_subscription<std_msgs::msg::Empty>(
        "/veye/mapping_done",
        10,
        std::bind(&MonocularSlamNode::OnMappingDone, this, std::placeholders::_1));

    m_tracking_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
        "/orb_slam3/tracking_image", 10);
    m_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/orb_slam3/camera_pose", 10);
    m_map_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_slam3/map_points", 10);
    m_tracking_state_pub = this->create_publisher<std_msgs::msg::UInt8>(
        "/orb_slam3/tracking_state", 10);
    m_keyframe_pub = this->create_publisher<veye_msgs::msg::Keyframe>(
        "/veye/keyframe", 10);
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
        this->get_logger(),
        "ORB-SLAM3 mono node ready. Map output: %s",
        m_map_output_dir.c_str());
}

MonocularSlamNode::~MonocularSlamNode()
{
    SaveMapArtifacts();
    m_SLAM->Shutdown();
}

void MonocularSlamNode::OnMappingDone(const std_msgs::msg::Empty::SharedPtr)
{
    RCLCPP_INFO(this->get_logger(), "Mapping finished, saving map and shutting down...");
    SaveMapArtifacts();
    rclcpp::shutdown();
}

void MonocularSlamNode::SaveMapArtifacts()
{
    if (m_map_saved) {
        return;
    }

    std::error_code ec;
    std::filesystem::create_directories(m_map_output_dir, ec);
    if (ec) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to create map directory %s: %s",
            m_map_output_dir.c_str(),
            ec.message().c_str());
        return;
    }

    const std::string trajectory_path = m_map_output_dir + "/KeyFrameTrajectory.txt";
    const std::string pcd_path = m_map_output_dir + "/MapPoints.pcd";
    const std::string db_path = m_map_output_dir + "/MapDatabase.bin";

    m_SLAM->SaveKeyFrameTrajectoryTUM(trajectory_path);

    const auto map_points = m_SLAM->GetAllMapPoints();
    auto pts = MapPointCloud::CollectValidPoints(map_points);
    auto entries = MapDatabase::CollectEntries(map_points);

    if (entries.size() < m_best_entries.size()) {
        pts = m_best_pts;
        entries = m_best_entries;
        RCLCPP_WARN(
            this->get_logger(),
            "Current map empty or smaller than best snapshot; saving best snapshot with %zu points",
            entries.size());
    }

    if (MapPointCloud::SaveToPCD(pcd_path, pts)) {
        RCLCPP_INFO(this->get_logger(), "Saved %zu map points to %s", pts.size(), pcd_path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save %s", pcd_path.c_str());
    }

    if (MapDatabase::SaveToBinary(db_path, entries)) {
        RCLCPP_INFO(
            this->get_logger(),
            "Saved %zu descriptors to %s",
            entries.size(),
            db_path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save %s", db_path.c_str());
    }

    m_map_saved = true;
}

void MonocularSlamNode::UpdateBestMapSnapshot()
{
    const auto map_points = m_SLAM->GetAllMapPoints();
    const auto entries = MapDatabase::CollectEntries(map_points);
    if (entries.size() <= m_best_map_point_count) {
        return;
    }

    m_best_pts = MapPointCloud::CollectValidPoints(map_points);
    m_best_entries = entries;
    m_best_map_point_count = entries.size();
}

bool MonocularSlamNode::ShouldEmitKeyframe(const Sophus::SE3f& Twc, double timestamp_sec)
{
    if ((timestamp_sec - m_last_keyframe_emit_sec) < m_keyframe_interval_sec) {
        return false;
    }

    if (!m_has_last_keyframe_pose) {
        return true;
    }

    const Sophus::SE3f delta = m_last_keyframe_twc.inverse() * Twc;
    const double translation = delta.translation().norm();
    const double rotation = delta.so3().log().norm();
    return translation >= m_keyframe_min_translation || rotation >= m_keyframe_min_rotation_rad;
}

void MonocularSlamNode::MaybePublishKeyframe(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const Sophus::SE3f& Twc,
    int tracking_state)
{
    if (tracking_state != 2 && tracking_state != 5) {
        return;
    }

    const double timestamp_sec = Utility::StampToSec(msg->header.stamp);
    if (!ShouldEmitKeyframe(Twc, timestamp_sec)) {
        return;
    }

    veye_msgs::msg::Keyframe keyframe_msg;
    keyframe_msg.header = msg->header;
    keyframe_msg.image = *msg;
    keyframe_msg.slam_pose = Se3ToPose(Twc);
    keyframe_msg.tracking_state = tracking_state;
    m_keyframe_pub->publish(keyframe_msg);

    m_last_keyframe_emit_sec = timestamp_sec;
    m_last_keyframe_twc = Twc;
    m_last_keyframe_time_sec = timestamp_sec;
    m_has_last_keyframe_pose = true;
}

void MonocularSlamNode::PublishTrackingState(int tracking_state)
{
    std_msgs::msg::UInt8 state_msg;
    state_msg.data = static_cast<uint8_t>(tracking_state);
    m_tracking_state_pub->publish(state_msg);
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(
        m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    UpdateBestMapSnapshot();

    cv::Mat img_display = m_cvImPtr->image.clone();
    if (img_display.channels() == 1) {
        cv::cvtColor(img_display, img_display, cv::COLOR_GRAY2BGR);
    }

    std::vector<cv::KeyPoint> kps = m_SLAM->GetTrackedKeyPointsUn();
    for (auto& kp : kps) {
        cv::circle(img_display, kp.pt, 3, cv::Scalar(0, 255, 0), 1);
    }

    auto track_img_msg = cv_bridge::CvImage(msg->header, "bgr8", img_display).toImageMsg();
    m_tracking_image_pub->publish(*track_img_msg);

    const int tracking_state = m_SLAM->GetTrackingState();
    PublishTrackingState(tracking_state);

    if (tracking_state == 2 || tracking_state == 5) {
        Sophus::SE3f Twc = Tcw.inverse();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = "map";
        pose_msg.pose = Se3ToPose(Twc);
        m_pose_pub->publish(pose_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = pose_msg.header;
        tf_msg.child_frame_id = "camera";
        tf_msg.transform.translation.x = pose_msg.pose.position.x;
        tf_msg.transform.translation.y = pose_msg.pose.position.y;
        tf_msg.transform.translation.z = pose_msg.pose.position.z;
        tf_msg.transform.rotation = pose_msg.pose.orientation;
        m_tf_broadcaster->sendTransform(tf_msg);

        MaybePublishKeyframe(msg, Twc, tracking_state);
    }

    PublishMapPoints(msg->header);
}

void MonocularSlamNode::PublishMapPoints(const std_msgs::msg::Header& header)
{
    const auto pts = MapPointCloud::CollectValidPoints(m_SLAM->GetAllMapPoints());
    m_map_points_pub->publish(MapPointCloud::ToPointCloud2(pts, header));
}
