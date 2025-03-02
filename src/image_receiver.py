#!/usr/bin/env python3
import rospy
import gi
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gi.repository import Gst

gi.require_version("Gst", "1.0")


class GStreamerROSImageReceiver:
    def __init__(self):
        rospy.init_node("gstreamer_ros_image_receiver", anonymous=True)
        Gst.init(None)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/received_images", Image, queue_size=10)

        # Create GStreamer pipeline
        self.pipeline = Gst.parse_launch(
            "appsrc name=mysource is-live=true ! appsink name=mysink emit-signals=true"
        )

        self.appsink = self.pipeline.get_by_name("mysink")
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect("new-sample", self.on_new_sample)

        # Start pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_new_sample(self, sink):
        """
        Callback for handling new frames from GStreamer pipeline.
        """
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            _, height, width = caps.get_structure(0).get_int(
                "height"
            ), caps.get_structure(0).get_int("width")

            # Convert buffer to numpy array
            success, map_info = buf.map(Gst.MapFlags.READ)
            if success:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(
                    height, width, 3
                )
                buf.unmap(map_info)

                # Convert to ROS Image message and publish
                ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(ros_img)

        return Gst.FlowReturn.OK

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)


def main():
    receiver = GStreamerROSImageReceiver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
