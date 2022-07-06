from typing import Optional

import rospy
from std_msgs.msg import String, Bool, Float32
from hri_msgs.msg import EngagementLevel
from geometry_msgs.msg import TransformStamped

from tf2_ros import LookupException


PERSON_TF_TIMEOUT = rospy.Duration(0.01)


class Person:
    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/persons/" + id
        self.frame = "person_" + id

        # these 3 member variables are set when the Person instance is constructed in hri.py
        self.faces_ = {}
        self.bodies_ = {}
        self.voices_ = {}

        from . import Face, Body, Voice

        self.face_id: Optional[str] = None
        self.face: Optional[Face] = None
        self.body_id: Optional[str] = None
        self.body: Optional[Body] = None
        self.voice_id: Optional[str] = None
        self.voice: Optional[Voice] = None

        self.anonymous: bool = False
        self.engagement_status: Optional[EngagementLevel] = None
        self.alias: Optional[str] = None
        self.loc_confidence: float = 0.0

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
        self.face_id_sub.unregister()
        self.body_id_sub.unregister()
        self.voice_id_sub.unregister()
        self.anonymous_sub.unregister()
        self.alias_sub.unregister()
        self.engagement_sub.unregister()
        self.loc_confidence_sub.unregister()

    def on_face_id(self, msg):
        if msg.data and msg.data in self.faces_:
            self.face_id = msg.data
            self.face = self.faces_[self.face_id]
        else:
            self.face_id = None
            self.face = None

    def on_body_id(self, msg):
        if msg.data and msg.data in self.bodies_:
            self.body_id = msg.data
            self.body = self.bodies_[self.body_id]
        else:
            self.body_id = None
            self.body = None

    def on_voice_id(self, msg):
        if msg.data and msg.data in self.voices_:
            self.voice_id = msg.data
            self.voice = self.voices_[self.voice_id]
        else:
            self.voice_id = None
            self.voice = None

    def on_anonymous(self, msg):
        self.anonymous = msg.data

    def on_alias(self, msg):
        self.alias = msg.data

    def on_engagement_status(self, msg):
        self.engagement_status = msg.data

    def on_loc_confidence(self, msg):
        self.loc_confidence = msg.data

    def face(self):
        pass

    def body(self):
        pass

    def voice(self):
        pass

    def engagement_level(self):
        if self.engagement_status is None:
            return None

        return self.engagement_status.level

    def transform(self):
        if self.loc_confidence == 0.0:
            return TransformStamped()

        try:
            return self._tf_buffer.lookupTransform(
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
