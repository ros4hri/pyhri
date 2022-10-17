from typing import Optional

try:
    import rospy
    from sensor_msgs.msg import RegionOfInterest, Image
    from cv_bridge import CvBridge
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


from .face import Rect


class Body:
    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/bodies/" + id

        self._valid = True

        self.roi: Optional[Rect] = None
        self.cropped = None

        self.cv_bridge = CvBridge()

        rospy.logdebug("New body detected: " + self.ns)

        self.roi_sub = rospy.Subscriber(self.ns + "/roi", RegionOfInterest, self.on_roi)

        self.cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self.on_cropped
        )

    def close(self):
        self._valid = False
        self.roi_sub.unregister()
        self.cropped_sub.unregister()

    def valid(self) -> bool:
        """Returns True if this body is still detected (and thus is valid).
        If False, methods like `Body.transform` will raise an exception.
        """
        return self._valid

    def on_roi(self, msg):
        self.roi = Rect(msg.x_offset, msg.y_offset, msg.width, msg.height)

    def on_cropped(self, msg):
        self.cropped = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def transform(self):
        raise NotImplementedError("method not yet implemented")

    def __str__(self):
        return self.id
