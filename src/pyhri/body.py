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
    from sensor_msgs.msg import RegionOfInterest, Image
    from hri_msgs.msg import Skeleton2D
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped

    from tf2_ros import LookupException

    BODY_TF_TIMEOUT = rospy.Duration(0.01)
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


from .face import Rect


class Body:
    """Represents a detected body."""

    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id  #: the body ID
        self.ns = "/humans/bodies/" + id  #: the full namespace of the body
        self.frame = "body_" + id  #: the body tf frame name

        self._valid = True

        self.roi: Optional[
            Rect
        ] = None  #: the face's region of interest (normalised between 0. and 1.) in the source image, if available
        self.cropped = (
            None  #: the cropped face image, as an OpenCV/numpy array, if available
        )

        self.skeleton2d: Optional[
            list
        ] = None  #: extracted 2D skeleton coordinate, using indices defined in `hri_msgs/Skeleton2D object <http://docs.ros.org/en/api/hri_msgs/html/msg/Skeleton2D.html>`_, if available

        self._cv_bridge = CvBridge()

        self._tf_buffer = tf_buffer
        self._reference_frame = reference_frame

        rospy.logdebug("New body detected: " + self.ns)

        self._roi_sub = rospy.Subscriber(
            self.ns + "/roi", RegionOfInterest, self._on_roi
        )

        self._cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self._on_cropped
        )

        self._skeleton2d_sub = rospy.Subscriber(
            self.ns + "/skeleton2d", Skeleton2D, self._on_skeleton2d
        )

    def close(self):
        self._valid = False
        self._roi_sub.unregister()
        self._cropped_sub.unregister()
        self._skeleton2d_sub.unregister()

    @property
    def valid(self) -> bool:
        """Returns `True` if this body is still detected (and thus is valid).
        If `False`, methods like `Body.transform` will raise an exception.
        """
        return self._valid

    def _on_roi(self, msg):
        self.roi = Rect(msg.x_offset, msg.y_offset, msg.width, msg.height)

    def _on_cropped(self, msg):
        self.cropped = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def _on_skeleton2d(self, msg):
        self.skeleton2d = msg.skeleton

    def transform(self, from_frame=None):
        """Returns a ROS TransformStamped of the body, from the `from_frame` reference basis.
        If `from_frame` is not provided, uses the default `reference_frame` (usually `base_link`).
        """

        if from_frame is None:
            from_frame = self._reference_frame

        try:
            return self._tf_buffer.lookup_transform(
                from_frame, self.frame, rospy.Time(0), BODY_TF_TIMEOUT
            )

        except LookupException:
            rospy.logwarn(
                "failed to transform body frame "
                + self.frame
                + " to "
                + from_frame
                + ". Are the frames published?"
            )

            return TransformStamped()

    def __str__(self):
        return self.id
