#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import sys


def read_lidar_fifo(fifo_path, pub):
    """Reads LiDAR data from a FIFO and publishes it as a ROS PointCloud2 message."""
    with open(fifo_path, "rb") as fifo:
        while not rospy.is_shutdown():
            try:
                data = fifo.read()
                if len(data) == 0:
                    continue

                msg = PointCloud2()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "lidar_frame"
                msg.height = 1  # Assume organized point cloud
                msg.width = len(data) // 16  # Assume 16 bytes per point
                msg.is_dense = True
                msg.is_bigendian = False
                msg.point_step = 16
                msg.row_step = msg.width * msg.point_step
                msg.fields = []  # Define fields as needed
                msg.data = data  # Raw data assignment

                pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"Error processing LiDAR data: {e}")
                break


def main():
    rospy.init_node("lidar_fifo_publisher", anonymous=True)
    pub = rospy.Publisher("/lidar_points_received", PointCloud2, queue_size=10)
    fifo_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/lidar_fifo"
    read_lidar_fifo(fifo_path, pub)


if __name__ == "__main__":
    main()
