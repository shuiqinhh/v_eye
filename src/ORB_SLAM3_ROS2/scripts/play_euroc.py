#!/usr/bin/env python3
"""
读取 EuRoC ROS2 bag 并将 /cam0/image_raw 发布到 /camera 话题
用法: python3 play_euroc.py <bag_path> [rate]
"""
import sys
import time
import sqlite3
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message

BAG_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/EuRoC/ros2bags/MH_01_easy/MH_01_easy.db3"
RATE = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0


class BagPlayer(Node):
    def __init__(self):
        super().__init__('bag_player')
        self.pub = self.create_publisher(Image, '/camera', 10)
        self.get_logger().info(f'Publishing /cam0/image_raw -> /camera at {RATE}x speed')

    def play(self):
        conn = sqlite3.connect(BAG_PATH)
        cur = conn.cursor()

        # 获取 /cam0/image_raw 的 topic id
        cur.execute("SELECT id FROM topics WHERE name='/cam0/image_raw'")
        row = cur.fetchone()
        if row is None:
            self.get_logger().error('Topic /cam0/image_raw not found in bag!')
            return
        topic_id = row[0]

        # 按时间戳顺序读取消息
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,)
        )
        rows = cur.fetchall()
        self.get_logger().info(f'Found {len(rows)} image messages, starting playback...')

        prev_ts = None
        for ts, data in rows:
            if not rclpy.ok():
                break
            if prev_ts is not None:
                sleep_time = (ts - prev_ts) * 1e-9 / RATE
                if sleep_time > 0:
                    time.sleep(sleep_time)
            prev_ts = ts

            msg = deserialize_message(data, Image)
            self.pub.publish(msg)

        conn.close()
        self.get_logger().info('Playback finished.')


def main():
    rclpy.init()
    player = BagPlayer()
    try:
        player.play()
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
