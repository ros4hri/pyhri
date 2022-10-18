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
    from cv_bridge import CvBridge
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


from .face import Rect


class Body:
    """Represents a detected body."""

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
