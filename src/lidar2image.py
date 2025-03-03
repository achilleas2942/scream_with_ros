#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge


def range_projection(
    points: np.ndarray,
    proj_fov_up: float = 3.0,
    proj_fov_down: float = -24.9,
    proj_W=2048,
    proj_H=64,
    remissions: np.ndarray = None,
):
    """
    Convert LiDAR point cloud to 2D range image.

    Project a point cloud into a spherical (range) image.

    Args:
        points (np.ndarray): Array of shape (N, 3) containing XYZ coordinates.
        proj_fov_up (float): Lidar upper field of view in degrees.
        proj_fov_down (float): Lidar lower field of view in degrees.
        proj_W (int): Width of the projected range image.
        proj_H (int): Height of the projected range image.
        remissions (np.ndarray, optional): Array of shape (N,) of remission values.

    Returns:
        proj_range (np.ndarray): The resulting 2D range image (proj_H x proj_W).
    """

    if remissions is None:
        # If no remissions provided, just use zeros
        remissions = np.zeros(points.shape[0], dtype=np.float32)

    fov_up_rad = proj_fov_up / 180.0 * np.pi
    fov_down_rad = proj_fov_down / 180.0 * np.pi

    fov_rad = abs(fov_down_rad) + abs(fov_up_rad)

    depth = np.linalg.norm(points, axis=1)  # shape: (N,)

    scan_x = points[:, 0]
    scan_y = points[:, 1]
    scan_z = points[:, 2]

    yaw = -np.arctan2(scan_y, scan_x)
    pitch = np.arcsin(scan_z / (depth + 1e-8))

    proj_x = 0.5 * (yaw / np.pi + 1.0)  # in [0.0, 1.0]
    proj_y = 1.0 - (pitch + abs(fov_down_rad)) / fov_rad  # in [0.0, 1.0]

    proj_x *= proj_W  # [0, W]
    proj_y *= proj_H  # [0, H]

    proj_x = np.floor(proj_x).astype(np.int32)
    proj_y = np.floor(proj_y).astype(np.int32)
    proj_x = np.clip(proj_x, 0, proj_W - 1)
    proj_y = np.clip(proj_y, 0, proj_H - 1)
    proj_range = np.zeros((proj_H, proj_W), dtype=np.float32)

    order = np.argsort(depth)[::-1]
    depth_sorted = depth[order]
    px_sorted = proj_x[order]
    py_sorted = proj_y[order]
    proj_range[py_sorted, px_sorted] = depth_sorted

    return proj_range


def undo_projection(
    range_image: np.ndarray,
    proj_fov_up: float = 3.0,
    proj_fov_down: float = -24.9,
    proj_W: int = 2048,
    proj_H: int = 64,
):
    """
    Undo the projection of a range image back to a point cloud.

    Args:
        range_image (np.ndarray): The range image to be converted back to a point cloud.
        proj_fov_up (float): Lidar upper field of view in degrees.
        proj_fov_down (float): Lidar lower field of view in degrees.
        proj_W (int): Width of the projected range image.
        proj_H (int): Height of the projected range image.
    Returns:
        point_cloud (np.ndarray): The resulting point cloud (N x 3).
    """

    fov_up = proj_fov_up / 180.0 * np.pi  # field of view up in radians
    fov_down = proj_fov_down / 180.0 * np.pi  # field of view down in radians
    fov = abs(fov_down) + abs(fov_up)  # total field of view in radians

    u = np.linspace(0, proj_W - 1, proj_W)  # horizontal indices
    v = np.linspace(0, proj_H - 1, proj_H)  # vertical indices
    grid_u, grid_v = np.meshgrid(u, v)

    proj_x = grid_u / proj_W  # normalized [0, 1]
    proj_y = grid_v / proj_H  # normalized [0, 1]

    # Revert the normalization to angles
    yaw = -(proj_x * 2.0 - 1.0) * np.pi  # yaw in [-pi, pi]
    pitch = (1.0 - proj_y) * fov - abs(fov_down)  # pitch in [fov_down, fov_up]

    depth = range_image[grid_v.astype(int), grid_u.astype(int)]

    scan_x = depth * np.cos(pitch) * np.cos(yaw)
    scan_y = depth * np.cos(pitch) * np.sin(yaw)
    scan_z = depth * np.sin(pitch)

    point_cloud = np.stack((scan_x, scan_y, scan_z), axis=-1).reshape(-1, 3)

    valid_mask = (depth > 0) & np.isfinite(depth)
    point_cloud = point_cloud[valid_mask.ravel()]

    return point_cloud


class LidarToImage:
    def __init__(self):
        rospy.init_node("lidar2image_node", anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/images", Image, queue_size=10)
        self.sub = rospy.Subscriber("/lidar_points", PointCloud2, self.callback)

    def callback(self, msg):
        points = np.array(
            [
                p[:3]
                for p in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            ]
        )
        if points.shape[0] == 0:
            return

        img = range_projection(points)

        # Convert grayscale to 3-channel image
        img_color = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        img_msg = self.bridge.cv2_to_imgmsg(img_color, encoding="bgr8")

        self.pub.publish(img_msg)


if __name__ == "__main__":
    LidarToImage()
    rospy.spin()
