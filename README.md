source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

ros2 run orbslam3 mono \
  /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt \
  /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/config/monocular/EuRoC.yaml
  false

python3 /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/scripts/play_euroc.py \
  /home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/EuRoC/ros2bags/MH_01_easy/MH_01_easy.db3 \
  1.0

ros2 launch veye_local offline_mapping.launch.py
ros2 launch veye_local online_localization.launch.py
ros2 run rqt_image_view rqt_image_view /orb_slam3/tracking_image
