#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "veye_msgs/msg/keyframe.hpp"

#include <Eigen/Core>
#include "map_database.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "System.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void PublishMapPoints(const std_msgs::msg::Header& header);
    void PublishTrackingState(int tracking_state);
    void MaybePublishKeyframe(
        const sensor_msgs::msg::Image::SharedPtr msg,
        const Sophus::SE3f& Twc,
        int tracking_state);
    void SaveMapArtifacts();
    bool ShouldEmitKeyframe(const Sophus::SE3f& Twc, double timestamp_sec);
    void OnMappingDone(const std_msgs::msg::Empty::SharedPtr msg);
    void UpdateBestMapSnapshot();

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_mapping_done_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_tracking_image_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_points_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_tracking_state_pub;
    rclcpp::Publisher<veye_msgs::msg::Keyframe>::SharedPtr m_keyframe_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    std::string m_map_output_dir;
    double m_keyframe_interval_sec;
    double m_keyframe_min_translation;
    double m_keyframe_min_rotation_rad;

    bool m_has_last_keyframe_pose;
    Sophus::SE3f m_last_keyframe_twc;
    double m_last_keyframe_time_sec;
    double m_last_keyframe_emit_sec;

    bool m_map_saved;

    std::vector<Eigen::Vector3f> m_best_pts;
    std::vector<MapDatabase::MapPointEntry> m_best_entries;
    size_t m_best_map_point_count;
};

#endif
