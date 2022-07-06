from typing import Optional
import numpy.typing as npt

import rospy
from sensor_msgs.msg import RegionOfInterest, Image
from hri_msgs.msg import FacialLandmarks, SoftBiometrics
from cv_bridge import CvBridge


class Rect:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.width = w
        self.height = h


class Face:
    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/faces/" + id

        self.roi: Optional[Rect] = None
        self.cropped: Optional[npt.ArrayLike] = None
        self.aligned: Optional[npt.ArrayLike] = None
        self.landmarks: Optional[FacialLandmarks] = None
        self.softbiometrics: Optional[SoftBiometrics] = None

        self.cv_bridge = CvBridge()

        rospy.logdebug("New face detected: " + self.ns)

        self.roi_sub = rospy.Subscriber(self.ns + "/roi", RegionOfInterest, self.on_roi)

        self.cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self.on_cropped
        )

        self.aligned_sub = rospy.Subscriber(
            self.ns + "/aligned", Image, self.on_aligned
        )

        self.landmarks_sub = rospy.Subscriber(
            self.ns + "/landmarks", FacialLandmarks, self.on_landmarks
        )

        self.softbiometrics_sub = rospy.Subscriber(
            self.ns + "/softbiometrics", SoftBiometrics, self.on_softbiometrics
        )

    def close(self):
        self.roi_sub.unregister()
        self.cropped_sub.unregister()
        self.aligned_sub.unregister()
        self.landmarks_sub.unregister()
        self.softbiometrics_sub.unregister()

    def on_roi(self, msg):
        self.roi = Rect(msg.x_offset, msg.y_offset, msg.width, msg.height)

    def on_cropped(self, msg):
        self.cropped = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def on_aligned(self, msg):
        self.aligned = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def on_landmarks(self, msg):
        self.landmarks = msg

    def on_softbiometrics(self, msg):
        self.softbiometrics = msg

    def __str__(self):
        return self.id
