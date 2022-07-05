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

        self.face_id: Optional[str] = None
        self.body_id: Optional[str] = None
        self.voice_id: Optional[str] = None

        self.anonymous: bool = False
        self.engagement_status: Optional[EngagementLevel] = None
        self.alias: Optional[str] = None
        self.loc_confidence: float = 0.0

        self.tf_buffer = tf_buffer
        self.reference_frame = reference_frame

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

    def on_face_id(self, msg):
        self.face_id = msg.data

    def on_body_id(self, msg):
        self.body = msg.data

    def on_voice_id(self, msg):
        self.voice_id = msg.data

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
            return self.tf_buffer.lookupTransform(
                self.reference_frame, self.frame, rospy.Time(0), PERSON_TF_TIMEOUT
            )

        except LookupException:
            rospy.logwarn(
                "failed to transform person frame "
                + self.frame
                + " to "
                + self.reference_frame
                + ". Are the frames published?"
            )

            return TransformStamped()
