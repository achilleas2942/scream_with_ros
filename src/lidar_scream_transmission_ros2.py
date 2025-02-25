#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import gi
import numpy as np
from gi.repository import Gst

gi.require_version('Gst', '1.0')

class GStreamerROS2LidarBridge(Node):
    def __init__(self, topic_name):
        super().__init__('gstreamer_ros2_lidar_bridge')

        # Initialize GStreamer
        Gst.init(None)

        # Define the GStreamer pipeline for streaming serialized LiDAR data
        pipeline_str = """
            appsrc name=mysource is-live=true format=TIME caps=application/x-raw,media=lidar !
            queue ! udpsink host=127.0.0.1 port=4000
        """
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("mysource")

        # Subscribe to the ROS 2 PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2, topic_name, self.lidar_callback, 10
        )

        # Start the GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

    def lidar_callback(self, msg):
        """
        Callback function to handle incoming ROS 2 PointCloud2 messages.
        """
        try:
            # Convert the PointCloud2 message into a binary format
            lidar_data = np.frombuffer(msg.data, dtype=np.uint8)
            
            # Create a GStreamer buffer from the serialized data
            buf = Gst.Buffer.new_wrapped(lidar_data.tobytes())
            
            # Push the buffer into the appsrc element
            retval = self.appsrc.emit("push-buffer", buf)
            if retval != Gst.FlowReturn.OK:
                self.get_logger().error(f"Failed to push buffer: {retval}")

        except Exception as e:
            self.get_logger().error(f"Error processing PointCloud2: {e}")

    def stop(self):
        # Stop the GStreamer pipeline on shutdown
        self.pipeline.set_state(Gst.State.NULL)

def main():
    # Initialize ROS 2
    rclpy.init()

    # Create the node and pass the topic name
    topic_name = 'lidar_points'
    node = GStreamerROS2LidarBridge(topic_name)

    try:
        # Spin the node to process ROS 2 messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
