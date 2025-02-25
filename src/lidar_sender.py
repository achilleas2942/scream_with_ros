#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import gi
import numpy as np
from gi.repository import Gst

gi.require_version('Gst', '1.0')

class GStreamerROS1LidarBridge:
    def __init__(self, topic_name):
        rospy.init_node('gstreamer_ros1_lidar_bridge', anonymous=True)

        # Initialize GStreamer
        Gst.init(None)

        # Define the GStreamer pipeline for streaming serialized LiDAR data
        pipeline_str = """
            appsrc name=mysource is-live=true format=TIME caps=application/x-raw,media=lidar !
            queue ! udpsink host=127.0.0.1 port=4000
        """
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("mysource")

        # Subscribe to the ROS 1 PointCloud2 topic
        self.subscription = rospy.Subscriber(topic_name, PointCloud2, self.lidar_callback)

        # Start the GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

    def lidar_callback(self, msg):
        """
        Callback function to handle incoming ROS 1 PointCloud2 messages.
        """
        try:
            # Convert the PointCloud2 message into a binary format
            lidar_data = np.frombuffer(msg.data, dtype=np.uint8)
            
            # Create a GStreamer buffer from the serialized data
            buf = Gst.Buffer.new_wrapped(lidar_data.tobytes())
            
            # Push the buffer into the appsrc element
            retval = self.appsrc.emit("push-buffer", buf)
            if retval != Gst.FlowReturn.OK:
                rospy.logerr(f"Failed to push buffer: {retval}")

        except Exception as e:
            rospy.logerr(f"Error processing PointCloud2: {e}")

    def stop(self):
        # Stop the GStreamer pipeline on shutdown
        self.pipeline.set_state(Gst.State.NULL)

def main():
    topic_name = 'lidar_points'
    bridge = GStreamerROS1LidarBridge(topic_name)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
