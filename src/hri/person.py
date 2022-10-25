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
    from std_msgs.msg import String, Bool, Float32
    from hri_msgs.msg import EngagementLevel
    from geometry_msgs.msg import TransformStamped

    from tf2_ros import LookupException

    PERSON_TF_TIMEOUT = rospy.Duration(0.01)
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


class Person:
    """Represents a known person (that can be currently tracked or not).

    A :class:`Person` instance gives access to its corresponding :class:`Face`,
    :class:`Body` and :class:`Voice` if available.
    """

    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id  #: the person ID
        self.ns = "/humans/persons/" + id  #: the person namespace
        self.frame = "person_" + id  #: the TF frame of the person

        self._valid = True

        # set in hri.py upon construction
        self._hrilistener = None

        from . import Face, Body, Voice

        self.face_id: Optional[str] = None
        self.face: Optional[
            Face
        ] = None  #: access to the person's face (:class:`Face` instance), if available
        self.body_id: Optional[str] = None
        self.body: Optional[
            Body
        ] = None  #: access to the person's body (:class:`Body` instance), if available
        self.voice_id: Optional[str] = None
        self.voice: Optional[
            Voice
        ] = None  #: access to the person's voice (:class:`Voice` instance), if available

        self.anonymous: bool = False  #: whether or not this person is anonymous
        self.engagement_status: Optional[
            EngagementLevel
        ] = None  #: current estimated level of engagement
        self.alias: Optional[
            str
        ] = None  #: if non-null, the person this person is an alias of
        self.loc_confidence: float = 0.0  #: level of confidence of the person's location (0.0: no confidence, 1.0: person currently seen)

        self._tf_buffer = tf_buffer
        self._reference_frame = reference_frame

        rospy.logdebug("New person detected: " + self.ns)

        self.face_id_sub = rospy.Subscriber(
            self.ns + "/face_id", String, self.on_face_id
        )

        self.body_id_sub = rospy.Subscriber(
            self.ns + "/body_id", String, self.on_body_id
        )

        self.voice_id_sub = rospy.Subscriber(
            self.ns + "/voice_id", String, self.on_voice_id
        )

        self.anonymous_sub = rospy.Subscriber(
            self.ns + "/anonymous", Bool, self.on_anonymous
        )

        self.alias_sub = rospy.Subscriber(self.ns + "/alias", String, self.on_alias)

        self.engagement_sub = rospy.Subscriber(
            self.ns + "/engagement_status", EngagementLevel, self.on_engagement_status
        )

        self.loc_confidence_sub = rospy.Subscriber(
            self.ns + "/location_confidence", Float32, self.on_loc_confidence
        )

    def close(self):
        self._valid = False
        self.face_id_sub.unregister()
        self.body_id_sub.unregister()
        self.voice_id_sub.unregister()
        self.anonymous_sub.unregister()
        self.alias_sub.unregister()
        self.engagement_sub.unregister()
        self.loc_confidence_sub.unregister()

    @property
    def valid(self) -> bool:
        """Returns True if this person still exists (and thus is valid).
        If False, methods like `Person.transform` will raise an exception.
        """
        return self._valid

    def on_face_id(self, msg):
        if msg.data and msg.data in self._hrilistener._faces:
            self.face_id = msg.data
            self.face = self._hrilistener._faces[self.face_id]
        else:
            self.face_id = None
            self.face = None

    def on_body_id(self, msg):
        if msg.data and msg.data in self._hrilistener._bodies:
            self.body_id = msg.data
            self.body = self._hrilistener._bodies[self.body_id]
        else:
            self.body_id = None
            self.body = None

    def on_voice_id(self, msg):
        if msg.data and msg.data in self._hrilistener._voices:
            self.voice_id = msg.data
            self.voice = self._hrilistener._voices[self.voice_id]
        else:
            self.voice_id = None
            self.voice = None

    def on_anonymous(self, msg):
        self.anonymous = msg.data

    def on_alias(self, msg):
        self.alias = msg.data

    def on_engagement_status(self, msg):
        self.engagement_status = msg.level

    def on_loc_confidence(self, msg):
        self.loc_confidence = msg.data

    def engagement_level(self):
        if self.engagement_status is None:
            return None

        return self.engagement_status.level

    def transform(self):
        if self.loc_confidence == 0.0:
            return TransformStamped()

        try:
            return self._tf_buffer.lookup_transform(
                self._reference_frame, self.frame, rospy.Time(0), PERSON_TF_TIMEOUT
            )

        except LookupException:
            rospy.logwarn(
                "failed to transform person frame "
                + self.frame
                + " to "
                + self._reference_frame
                + ". Are the frames published?"
            )

            return TransformStamped()

    def __str__(self):
        return self.id
