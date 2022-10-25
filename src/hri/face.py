# Copyright 2022 PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PAL Robotics S.L. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from typing import Optional

try:
    import rospy
    from sensor_msgs.msg import Image
    from hri_msgs.msg import (
        NormalizedRegionOfInterest2D,
        FacialLandmarks,
        SoftBiometrics,
    )
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped

    from tf2_ros import LookupException

    FACE_TF_TIMEOUT = rospy.Duration(0.01)
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


class Rect:
    def __init__(self, xmin, ymin, xmax, ymax):
        self.x = xmin
        self.y = ymin
        self.width = xmax - xmin
        self.height = ymax - ymin

        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax


class Face:
    """Represents a detected face."""

    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id  #: the face ID
        self.ns = "/humans/faces/" + id  #: the full namespace of the face
        self.frame = "face_" + id  #: the face tf frame name
        self.gaze_frame = "gaze_" + id  #: the corresponding gaze tf frame name

        self._valid = True

        self.roi: Optional[
            Rect
        ] = None  #: the face's region of interest (normalised between 0. and 1.) in the source image, if available
        self.cropped = (
            None  #: the cropped face image, as an OpenCV/numpy array, if available
        )
        self.aligned = None  #: the cropped and aligned (eg, the two eyes are aligned horizontally) face image, as an OpenCV/numpy array, if available
        self.landmarks: Optional[
            FacialLandmarks
        ] = None  #: extracted facial landmarks, as a `hri_msgs/FacialLnadmark object <http://docs.ros.org/en/api/hri_msgs/html/msg/FacialLandmarks.html>`_, if available
        self.softbiometrics: Optional[
            SoftBiometrics
        ] = None  #: estimated 'soft biometrics' (eg age, gender) of the face, as a `hri_msgs/SoftBiometrics object <http://docs.ros.org/en/api/hri_msgs/html/msg/SoftBiometrics.html>`_, if available

        self._cv_bridge = CvBridge()

        self._tf_buffer = tf_buffer
        self._reference_frame = reference_frame

        rospy.logdebug("New face detected: " + self.ns)

        self._roi_sub = rospy.Subscriber(
            self.ns + "/roi", NormalizedRegionOfInterest2D, self._on_roi
        )

        self._cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self._on_cropped
        )

        self._aligned_sub = rospy.Subscriber(
            self.ns + "/aligned", Image, self._on_aligned
        )

        self._landmarks_sub = rospy.Subscriber(
            self.ns + "/landmarks", FacialLandmarks, self._on_landmarks
        )

        self._softbiometrics_sub = rospy.Subscriber(
            self.ns + "/softbiometrics", SoftBiometrics, self._on_softbiometrics
        )

    def close(self):
        self._valid = False
        self._roi_sub.unregister()
        self._cropped_sub.unregister()
        self._aligned_sub.unregister()
        self._landmarks_sub.unregister()
        self._softbiometrics_sub.unregister()

    @property
    def valid(self) -> bool:
        """Returns `True` if this face still exists (and thus is valid).
        If `False`, methods like `Face.transform` will raise an exception.
        """
        return self._valid

    def _on_roi(self, msg):
        self.roi = Rect(msg.xmin, msg.ymin, msg.xmax, msg.ymax)

    def _on_cropped(self, msg):
        self.cropped = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def _on_aligned(self, msg):
        self.aligned = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def _on_landmarks(self, msg):
        self.landmarks = msg

    def _on_softbiometrics(self, msg):
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
