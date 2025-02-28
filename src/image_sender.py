#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import gi
import numpy as np
from gi.repository import Gst, GLib, GObject

gi.require_version("Gst", "1.0")


class GStreamerROSLidarBridge:
    def __init__(self, topic_name):
        rospy.init_node("gstreamer_ros_lidar_bridge", anonymous=True)

        # Initialize GStreamer
        Gst.init(None)

        # Define the GStreamer pipeline for streaming serialized LiDAR data
        pipeline_str = """
            appsrc name=mysource is-live=true format=TIME caps=video/x-raw,format=RGB,width=1280,height=720,framerate=30/1 !
            videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=3150
        """
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("mysource")

        # Subscribe to the ROS 1 Image topic
        self.subscription = rospy.Subscriber(
            topic_name, Image, self.image_callback
        )

        # Start the GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

    def image_callback(self, msg):
        """
        Callback function to handle incoming ROS Image messages.
        """
        try:
            # Convert the Image message into a binary format
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

            # Ensure the frame matches the pipeline's caps (e.g., resize if necessary)
            # In this example, we assume the image is already 320x240
            if frame.shape[0] != 720 or frame.shape[1] != 1280:
                rospy.logerr("Image size mismatch. Expected 320x240.")
                return

            # Create a GStreamer buffer from the frame
            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            
            # Push the buffer into the appsrc element
            retval = self.appsrc.emit("push-buffer", buf)
            if retval != Gst.FlowReturn.OK:
                rospy.logerr(f"Failed to push buffer: {retval}")

        except Exception as e:
            rospy.logerr(f"Error processing Image: {e}")

    def stop(self):
        # Stop the GStreamer pipeline on shutdown
        self.pipeline.set_state(Gst.State.NULL)


def main():
    topic_name = "image"
    bridge = GStreamerROSLidarBridge(topic_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
