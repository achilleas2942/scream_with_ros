#!/usr/bin/env python3
import rospy
import numpy as np
import gi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gi.repository import Gst

gi.require_version("Gst", "1.0")


class GStreamerROSLidarBridge:
    def __init__(self, topic_name):
        rospy.init_node("gstreamer_ros_image_sender", anonymous=True)

        # Initialize GStreamer
        Gst.init(None)
        self.bridge = CvBridge()

        # Define the GStreamer pipeline for streaming serialized LiDAR data
        pipeline_str = """
            appsrc name=mysource is-live=true format=TIME caps=video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 !
            videoconvert ! x264enc bitrate=500 tune=zerolatency speed-preset=ultrafast ! rtph264pay ! 
            udpsink host=127.0.0.1 port=3150
        """
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("mysource")
        self.subscription = rospy.Subscriber(topic_name, Image, self.image_callback)
        self.pipeline.set_state(Gst.State.PLAYING)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Ensure BGR format
            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            retval = self.appsrc.emit("push-buffer", buf)
            if retval != Gst.FlowReturn.OK:
                rospy.logerr(f"Failed to push buffer: {retval}")
        except Exception as e:
            rospy.logerr(f"Error processing Image: {e}")

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)


def main():
    bridge = GStreamerROSLidarBridge("/images")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
