#!/usr/bin/env python3
"""VB-GPS integrator: fuse SLAM relative pose with MBL global pose."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import UInt8
from tf2_ros import TransformBroadcaster

from veye_msgs.msg import FusedPose, GlobalPose


def pose_to_matrix(pose: Pose) -> np.ndarray:
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot = Rotation.from_quat(quat).as_matrix()
    trans = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = trans
    return mat


def matrix_to_pose(mat: np.ndarray) -> Pose:
    pose = Pose()
    rot = Rotation.from_matrix(mat[:3, :3]).as_quat()
    pose.orientation.x = float(rot[0])
    pose.orientation.y = float(rot[1])
    pose.orientation.z = float(rot[2])
    pose.orientation.w = float(rot[3])
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    return pose


class Integrator(Node):
    LOST_STATES = {3, 4}

    def __init__(self) -> None:
        super().__init__("veye_integrator")

        self.scale = 1.0
        self.scale_alpha = float(self.declare_parameter("scale_alpha", 0.2).value)
        self.min_motion = float(self.declare_parameter("min_motion", 0.02).value)
        self.smooth_alpha = float(self.declare_parameter("smooth_alpha", 0.35).value)

        self.slam_pose: np.ndarray | None = None
        self.prev_slam_pose: np.ndarray | None = None
        self.mbl_pose: np.ndarray | None = None
        self.prev_mbl_pose: np.ndarray | None = None
        self.align_rot = np.eye(3, dtype=np.float64)
        self.align_trans = np.zeros(3, dtype=np.float64)
        self.initialized = False
        self.fused_pose: np.ndarray | None = None
        self.tracking_state = 0
        self.mbl_valid = False

        self.fused_pub = self.create_publisher(FusedPose, "/veye/fused_pose", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/veye/fused_pose_stamped", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(PoseStamped, "/orb_slam3/camera_pose", self.on_slam_pose, 10)
        self.create_subscription(GlobalPose, "/veye/global_pose", self.on_global_pose, 10)
        self.create_subscription(UInt8, "/orb_slam3/tracking_state", self.on_tracking_state, 10)

    def on_tracking_state(self, msg: UInt8) -> None:
        self.tracking_state = int(msg.data)

    def on_global_pose(self, msg: GlobalPose) -> None:
        self.mbl_valid = bool(msg.valid)
        if not msg.valid:
            return

        self.prev_mbl_pose = self.mbl_pose
        self.mbl_pose = pose_to_matrix(msg.pose)
        self._update_scale()
        self._update_alignment()
        self._publish_fused(msg.header.stamp)

    def on_slam_pose(self, msg: PoseStamped) -> None:
        self.prev_slam_pose = self.slam_pose
        self.slam_pose = pose_to_matrix(msg.pose)
        if self.mbl_valid and self.mbl_pose is not None:
            self._update_scale()
        self._publish_fused(msg.header.stamp)

    def _update_scale(self) -> None:
        if (
            self.prev_slam_pose is None
            or self.prev_mbl_pose is None
            or self.slam_pose is None
            or self.mbl_pose is None
        ):
            return

        ds = np.linalg.norm(self.slam_pose[:3, 3] - self.prev_slam_pose[:3, 3])
        dm = np.linalg.norm(self.mbl_pose[:3, 3] - self.prev_mbl_pose[:3, 3])
        if ds < self.min_motion or dm < self.min_motion:
            return

        target_scale = dm / ds
        self.scale = (1.0 - self.scale_alpha) * self.scale + self.scale_alpha * target_scale

    def _update_alignment(self) -> None:
        if self.slam_pose is None or self.mbl_pose is None:
            return

        scaled_slam_t = self.scale * self.slam_pose[:3, 3]
        slam_rot = self.slam_pose[:3, :3]
        mbl_rot = self.mbl_pose[:3, :3]
        mbl_t = self.mbl_pose[:3, 3]

        self.align_rot = mbl_rot @ slam_rot.T
        self.align_trans = mbl_t - self.align_rot @ scaled_slam_t
        self.initialized = True

    def _compute_fused(self) -> np.ndarray | None:
        if self.slam_pose is None:
            return self.mbl_pose

        if not self.initialized:
            if self.mbl_pose is None:
                return None
            return self.mbl_pose.copy()

        fused = np.eye(4, dtype=np.float64)
        fused[:3, :3] = self.align_rot @ self.slam_pose[:3, :3]
        fused[:3, 3] = self.align_rot @ (self.scale * self.slam_pose[:3, 3]) + self.align_trans
        return fused

    def _publish_fused(self, stamp) -> None:
        fused = self._compute_fused()
        if fused is None:
            return

        if self.fused_pose is None:
            self.fused_pose = fused
        else:
            blended = self.fused_pose.copy()
            blended[:3, 3] = (
                (1.0 - self.smooth_alpha) * self.fused_pose[:3, 3]
                + self.smooth_alpha * fused[:3, 3]
            )
            blended[:3, :3] = fused[:3, :3]
            self.fused_pose = blended

        out = FusedPose()
        out.header.stamp = stamp
        out.header.frame_id = "map"
        out.pose = matrix_to_pose(self.fused_pose)
        out.scale = float(self.scale)
        out.slam_lost = self.tracking_state in self.LOST_STATES
        out.mbl_valid = self.mbl_valid
        self.fused_pub.publish(out)

        pose_stamped = PoseStamped()
        pose_stamped.header = out.header
        pose_stamped.pose = out.pose
        self.pose_pub.publish(pose_stamped)

        tf_msg = TransformStamped()
        tf_msg.header = out.header
        tf_msg.child_frame_id = "veye_fused"
        tf_msg.transform.translation.x = out.pose.position.x
        tf_msg.transform.translation.y = out.pose.position.y
        tf_msg.transform.translation.z = out.pose.position.z
        tf_msg.transform.rotation = out.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = Integrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
