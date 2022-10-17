from typing import Optional

try:
    import rospy
    from sensor_msgs.msg import RegionOfInterest, Image
    from hri_msgs.msg import FacialLandmarks, SoftBiometrics
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped

    from tf2_ros import LookupException

    FACE_TF_TIMEOUT = rospy.Duration(0.01)
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


class Rect:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.width = w
        self.height = h


class Face:
    """Represents a detected face."""

    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/faces/" + id
        self.frame = "face_" + id
        self.gaze_frame = "gaze_" + id

        self._valid = True

        self.roi: Optional[Rect] = None
        self.cropped = None
        self.aligned = None
        self.landmarks: Optional[FacialLandmarks] = None
        self.softbiometrics: Optional[SoftBiometrics] = None

        self.cv_bridge = CvBridge()

        self._tf_buffer = tf_buffer
        self._reference_frame = reference_frame

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
        self._valid = False
        self.roi_sub.unregister()
        self.cropped_sub.unregister()
        self.aligned_sub.unregister()
        self.landmarks_sub.unregister()
        self.softbiometrics_sub.unregister()

    def valid(self) -> bool:
        """Returns True if this face still exists (and thus is valid).
        If False, methods like `Face.transform` will raise an exception.
        """
        return self._valid

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

    def transform(self, from_frame=None):
        """Returns a ROS TransformStamped of the face, from the `from_frame` reference basis.
        If `from_frame` is not provided, uses the default `reference_frame` (usually `base_link`).
        """

        if from_frame is None:
            from_frame = self._reference_frame

        try:
            return self._tf_buffer.lookup_transform(
                from_frame, self.frame, rospy.Time(0), FACE_TF_TIMEOUT
            )

        except LookupException:
            rospy.logwarn(
                "failed to transform face frame "
                + self.frame
                + " to "
                + from_frame
                + ". Are the frames published?"
            )

            return TransformStamped()

    def gaze_transform(self, from_frame=None):
        """Returns a ROS TransformStamped of the face's gaze, from the `from_frame` reference basis.
        If `from_frame` is not provided, uses the default `reference_frame` (usually `base_link`).
        """

        if from_frame is None:
            from_frame = self._reference_frame

        try:
            return self._tf_buffer.lookup_transform(
                from_frame, self.gaze_frame, rospy.Time(0), FACE_TF_TIMEOUT
            )

        except LookupException:
            rospy.logwarn(
                "failed to transform gaze frame "
                + self.gaze_frame
                + " to "
                + from_frame
                + ". Are the frames published?"
            )

            return TransformStamped()

    def __str__(self):
        return self.id
