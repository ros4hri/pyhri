from typing import Optional
import cv2
import numpy.typing as npt

import rospy
from sensor_msgs.msg import RegionOfInterest, Image


class Body:
    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/bodies/" + id

        self.roi: Optional[cv2.Rect] = None
        self.cropped: Optional[npt.ArrayLike] = None

        rospy.logdebug("New body detected: " + self.ns)

        self.roi_sub = rospy.Subscriber(self.ns + "/roi", RegionOfInterest, self.on_roi)

        self.cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self.on_cropped
        )

    def close(self):
        self.roi_sub.unregister()
        self.cropped_sub.unregister()

    def on_roi(self, msg):
        self.roi = cv2.Rect(msg.x_offset, msg.y_offset, msg.width, msg.height)

    def on_cropped(self, msg):
        self.cropped = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
