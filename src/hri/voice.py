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
    from hri_msgs.msg import LiveSpeech
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import Bool

    from tf2_ros import LookupException

    VOICE_TF_TIMEOUT = rospy.Duration(0.01)
except ImportError:
    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


class Voice:
    """Represents a detected voice."""

    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id  #: the voice ID
        self.ns = "/humans/voices/" + id  #: the full namespace of the voice
        self.frame = "voice_" + id  #: the voice tf frame name

        self._valid = True

        self._tf_buffer = tf_buffer
        self._reference_frame = reference_frame

        self.incremental_speech: Optional[
            str
        ] = None  #: last incrementally recognised speech -- that string might entirely change, based on the what the ASR recognised

        self.speech: Optional[str] = None  #: last recognised speech, if available

        self.is_speaking: Optional[
            bool
        ] = None  #: whether speech is detected for this voice

        rospy.logdebug("New voice detected: " + self.ns)

        self._speech_sub = rospy.Subscriber(
            self.ns + "/speech", LiveSpeech, self._on_speech
        )

        self._is_speaking_sub = rospy.Subscriber(
            self.ns + "/is_speaking", Bool, self._on_is_speaking
        )

    def close(self):
        self._valid = False

    @property
    def valid(self) -> bool:
        """Returns `True` if this voice still exists (and thus is valid).
        If `False`, methods like `Voice.transform` will raise an exception.
        """
        return self._valid

    def _on_speech(self, msg):
        self.incremental_speech = msg.incremental
        self.speech = msg.final

    def _on_is_speaking(self, msg):
        self.is_speaking = msg.data

    def transform(self, from_frame=None):
        """Returns a ROS TransformStamped of the voice, from the `from_frame`
        reference basis.

        If `from_frame` is not provided, uses the default `reference_frame`
        (usually `base_link`).

        Note that the spatial location of a voice is inherently very
        approximate. Most sound source localisation algorithm will actually
        only return the 'DoA' (*direction of arrival*) of the voice -- in that
        case, only the heading of the tf frame with respect to the reference
        frame -- not the distance -- should be considered.
        """

        if from_frame is None:
            from_frame = self._reference_frame

        try:
            return self._tf_buffer.lookup_transform(
                from_frame, self.frame, rospy.Time(0), VOICE_TF_TIMEOUT
            )

        except LookupException:
            rospy.logwarn(
                "failed to transform voice frame "
                + self.frame
                + " to "
                + from_frame
                + ". Are the frames published?"
            )

            return TransformStamped()

    def __str__(self):
        return self.id
