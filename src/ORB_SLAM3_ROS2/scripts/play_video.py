#!/usr/bin/env python3
"""
从 MP4 视频读取帧并发布到 /camera，供 orbslam3 mono 节点使用。

用法:
  python3 play_video.py <video_path> [rate] [loop]

  rate  播放倍速，默认 1.0（原速）
  loop  播完后是否循环，默认 true
"""
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

VIDEO_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/videos/test_SLAM.mp4"
RATE = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
LOOP = sys.argv[3].lower() != "false" if len(sys.argv) > 3 else True


class VideoPlayer(Node):
    def __init__(self, video_path: str, rate: float, loop: bool):
        super().__init__("video_player")
        self.video_path = video_path
        self.rate = max(rate, 0.01)
        self.loop = loop
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera", 10)

    def play(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open video: {self.video_path}")
            return

        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_interval = 1.0 / fps / self.rate

        self.get_logger().info(
            f"Playing {self.video_path} ({width}x{height} @ {fps:.1f} fps) "
            f"-> /camera at {self.rate}x speed, loop={self.loop}"
        )

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                if self.loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.get_logger().info("Looping video...")
                    continue
                self.get_logger().info("Playback finished.")
                break

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.pub.publish(msg)
            time.sleep(frame_interval)

        cap.release()


def main():
    rclpy.init()
    player = VideoPlayer(VIDEO_PATH, RATE, LOOP)
    try:
        player.play()
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
