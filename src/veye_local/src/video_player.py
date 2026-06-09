#!/usr/bin/env python3
"""
从 MP4 视频读取帧并发布到 /camera，供 orbslam3 mono 节点使用。

与 scripts/play_video.py 行为对齐：
  - 等 ORB-SLAM3 就绪后再播（避免词袋加载期间丢帧）
  - 使用帧序号生成单调递增时间戳
  - 默认 QoS 与 play_video.py 一致

用法:
  ros2 run veye_local video_player <video_path> [rate] [loop]
"""

import sys
import time

import cv2
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


def stamp_from_seconds(stamp_sec: float) -> Time:
    msg = Time()
    msg.sec = int(stamp_sec)
    msg.nanosec = int(round((stamp_sec - msg.sec) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


class VideoPlayer(Node):
    def __init__(self, video_path: str, rate: float, loop: bool):
        super().__init__("video_player")
        self.video_path = video_path
        self.rate = max(rate, 0.01)
        self.loop = loop
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera", 10)
        self.done_pub = self.create_publisher(Empty, "/veye/mapping_done", 10)
        self._frame_idx = 0

    def _wait_for_slam(self) -> None:
        """Wait until mono node exists (same as manual: SLAM first, then video)."""
        self.get_logger().info("Waiting for ORB-SLAM3 node...")
        while rclpy.ok() and self.count_publishers("/orb_slam3/tracking_state") == 0:
            time.sleep(0.2)
        if not rclpy.ok():
            return
        # Node is constructed; brief pause so rclcpp::spin is processing callbacks.
        time.sleep(0.5)
        self.get_logger().info("ORB-SLAM3 ready, starting playback.")

    def play(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open video: {self.video_path}")
            return

        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_interval = 1.0 / fps / self.rate

        self._wait_for_slam()
        if not rclpy.ok():
            cap.release()
            return

        self.get_logger().info(
            f"Playing {self.video_path} ({width}x{height} @ {fps:.1f} fps) "
            f"-> /camera at {self.rate}x speed, loop={self.loop}"
        )

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                if self.loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self._frame_idx = 0
                    self.get_logger().info("Looping video...")
                    continue
                self.get_logger().info("Playback finished.")
                break

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp_from_seconds(self._frame_idx / fps)
            msg.header.frame_id = "camera"
            self.pub.publish(msg)
            self._frame_idx += 1
            time.sleep(frame_interval)

        cap.release()

        if not self.loop and rclpy.ok():
            # Allow SLAM to process the last frames before saving.
            time.sleep(1.0)
            self.done_pub.publish(Empty())
            self.get_logger().info("Notified SLAM that mapping is done.")
            time.sleep(0.5)


def main():
    video_path = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "/home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/videos/test_SLAM.mp4"
    )
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    loop = sys.argv[3].lower() != "false" if len(sys.argv) > 3 else True

    rclpy.init()
    player = VideoPlayer(video_path, rate, loop)
    try:
        player.play()
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
