#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from lidar2image import undo_projection


class ImageToLidar:
    def __init__(self):
        rospy.init_node("image2lidar", anonymous=True)
        self.bridge = CvBridge()
        self.lidar_pub = rospy.Publisher("/received_lidar_points", PointCloud2, queue_size=10)
        rospy.Subscriber("/received_images", Image, self.image_callback)

    def image_callback(self, msg):
        """
        Converts an image back to a LiDAR point cloud.
        """
        try:
            range_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            point_cloud = undo_projection(range_image)

            # Convert point cloud to ROS message
            header = msg.header
            header.frame_id = "lidar"
            cloud_msg = point_cloud2.create_cloud_xyz32(header, point_cloud.tolist())
            self.lidar_pub.publish(cloud_msg)

        except Exception as e:
            rospy.logerr(f"Error converting image to LiDAR: {e}")


def main():
    processor = ImageToLidar()
    rospy.spin()


if __name__ == "__main__":
    main()
