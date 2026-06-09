#!/usr/bin/env python3
"""Model-based localization against a pre-built ORB map database."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from veye_local.camera_utils import load_camera_yaml
from veye_local.map_database import load_map_database
from veye_msgs.msg import GlobalPose, Keyframe


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


def spatial_uniformity(
    keypoints: np.ndarray, image_width: int, image_height: int, grid: int = 4
) -> float:
    if keypoints.shape[0] == 0:
        return 0.0

    counts = np.zeros((grid, grid), dtype=np.int32)
    for x, y in keypoints:
        col = min(grid - 1, max(0, int(x / max(image_width, 1) * grid)))
        row = min(grid - 1, max(0, int(y / max(image_height, 1) * grid)))
        counts[row, col] += 1

    occupied = int(np.count_nonzero(counts))
    return occupied / float(grid * grid)


class MblLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("mbl_localizer")

        map_dir = self.declare_parameter("map_dir", "/home/rm/Desktop/ros2_ws/maps/default").value
        camera_yaml = self.declare_parameter(
            "camera_yaml",
            "/home/rm/Desktop/ros2_ws/src/ORB_SLAM3_ROS2/config/monocular/test.yaml",
        ).value
        self.min_inliers = int(self.declare_parameter("min_inliers", 12).value)
        self.ratio_test = float(self.declare_parameter("ratio_test", 0.75).value)
        self.reproj_error = float(self.declare_parameter("reproj_error", 8.0).value)
        self.confidence = float(self.declare_parameter("pnp_confidence", 0.99).value)
        self.min_uniformity = float(self.declare_parameter("min_uniformity", 0.25).value)

        self.intrinsics = load_camera_yaml(camera_yaml)
        db_path = f"{map_dir.rstrip('/')}/MapDatabase.bin"
        self.map_db = load_map_database(db_path)
        self.get_logger().info(f"Loaded {self.map_db.size} map points from {db_path}")

        self.orb = cv2.ORB_create(
            nfeatures=1500,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=20,
        )
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.bridge = CvBridge()

        self.pub = self.create_publisher(GlobalPose, "/veye/global_pose", 10)
        self.create_subscription(Keyframe, "/veye/keyframe", self.on_keyframe, 10)

    def localize(self, gray: np.ndarray) -> GlobalPose:
        msg = GlobalPose()
        msg.valid = False
        msg.confidence = 0.0
        msg.inlier_count = 0
        msg.spatial_uniformity = 0.0

        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) == 0:
            return msg

        matches = self.matcher.knnMatch(descriptors, self.map_db.descriptors, k=2)
        points_2d = []
        points_3d = []
        for pair in matches:
            if len(pair) < 2:
                continue
            best, second = pair
            if best.distance >= self.ratio_test * second.distance:
                continue
            kp = keypoints[best.queryIdx].pt
            points_2d.append(kp)
            points_3d.append(self.map_db.points[best.trainIdx])

        if len(points_2d) < self.min_inliers:
            msg.inlier_count = len(points_2d)
            return msg

        object_points = np.asarray(points_3d, dtype=np.float64)
        image_points = np.asarray(points_2d, dtype=np.float64)
        msg.spatial_uniformity = float(
            spatial_uniformity(image_points, self.intrinsics.width, self.intrinsics.height)
        )

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            self.intrinsics.camera_matrix,
            self.intrinsics.dist_coeffs,
            iterationsCount=200,
            reprojectionError=self.reproj_error,
            confidence=self.confidence,
            flags=cv2.SOLVEPNP_EPNP,
        )

        if not ok or inliers is None or len(inliers) < self.min_inliers:
            msg.inlier_count = 0 if inliers is None else len(inliers)
            return msg

        inlier_2d = image_points[inliers[:, 0]]
        msg.spatial_uniformity = float(
            spatial_uniformity(inlier_2d, self.intrinsics.width, self.intrinsics.height)
        )
        if msg.spatial_uniformity < self.min_uniformity:
            msg.inlier_count = len(inliers)
            return msg

        rot, _ = cv2.Rodrigues(rvec)
        t_cam_world = -rot.T @ tvec
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot.T
        mat[:3, 3] = t_cam_world.reshape(3)

        msg.pose = matrix_to_pose(mat)
        msg.valid = True
        msg.inlier_count = int(len(inliers))
        msg.confidence = min(
            1.0,
            (msg.inlier_count / 80.0) * 0.6 + msg.spatial_uniformity * 0.4,
        )
        return msg

    def on_keyframe(self, keyframe: Keyframe) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(keyframe.image, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to decode keyframe image: {exc}")
            return

        if cv_image.ndim == 3:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_image

        result = self.localize(gray)
        result.header = keyframe.header
        result.header.frame_id = "map"
        self.pub.publish(result)

        if result.valid:
            self.get_logger().info(
                f"MBL OK: inliers={result.inlier_count}, "
                f"uniformity={result.spatial_uniformity:.2f}, "
                f"confidence={result.confidence:.2f}"
            )
        else:
            self.get_logger().warn(
                f"MBL failed: inliers={result.inlier_count}, "
                f"uniformity={result.spatial_uniformity:.2f}"
            )


def main() -> None:
    rclpy.init()
    node = MblLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
