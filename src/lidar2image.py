#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge

def range_projection(points: np.ndarray, proj_W=1280, proj_H=720):
    """
    Convert LiDAR point cloud to 2D range image.
    """
    depth = np.linalg.norm(points, axis=1)
    yaw = -np.arctan2(points[:, 1], points[:, 0])
    pitch = np.arcsin(points[:, 2] / (depth + 1e-8))

    proj_x = (yaw / np.pi + 1.0) * 0.5 * proj_W
    proj_y = (1.0 - (pitch + 0.4363) / 0.4975) * proj_H

    proj_x = np.clip(np.floor(proj_x).astype(int), 0, proj_W - 1)
    proj_y = np.clip(np.floor(proj_y).astype(int), 0, proj_H - 1)

    proj_range = np.zeros((proj_H, proj_W), dtype=np.uint8)
    depth = np.clip(depth / np.max(depth) * 255, 0, 255).astype(np.uint8)

    proj_range[proj_y, proj_x] = depth
    return proj_range

class LidarToImage:
    def __init__(self):
        rospy.init_node("lidar_to_image_node", anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/images", Image, queue_size=10)
        self.sub = rospy.Subscriber("/lidar_points", PointCloud2, self.callback)

    def callback(self, msg):
        points = np.array([p[:3] for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
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
