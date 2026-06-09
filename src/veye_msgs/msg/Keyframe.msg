# Keyframe for MBL localization (published every 1-2 s from SLAM node)
std_msgs/Header header
sensor_msgs/Image image
geometry_msgs/Pose slam_pose
int32 tracking_state
