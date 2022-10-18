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

from typing import Mapping


from .body import Body
from .face import Face
from .voice import Voice
from .person import Person

try:
    import rospy
    from hri_msgs.msg import IdsList
    from tf2_ros import Buffer, TransformListener
except ImportError:

    class IdsList:
        pass

    print(
        "Importing pyhri without rospy! This won't work (except for generating documentation)"
    )


class HRIListener:
    """Main entry point to pyhri."""

    def __init__(self, reference_frame="base_link"):

        self.feature_subscribers_ = {}

        self.faces: Mapping[str, Face] = {}
        self.face_callbacks = []
        self.face_lost_callbacks = []

        self.bodies: Mapping[str, Body] = {}

        self.body_callbacks = []
        self.body_lost_callbacks = []

        self.voices: Mapping[str, Voice] = {}

        self.voice_callbacks = []
        self.voice_lost_callbacks = []

        self.tracked_persons: Mapping[str, Person] = {}

        self.person_tracked_callbacks = []
        self.person_tracked_lost_callbacks = []

        self.known_persons: Mapping[str, Person] = {}

        self.known_person_callbacks = []
        self.known_person_lost_callbacks = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)

        self._reference_frame = reference_frame

        rospy.logdebug("Initialising the Python HRI listener")

        self._subscribers = {
            "face": rospy.Subscriber(
                "/humans/faces/tracked", IdsList, self._on_tracked_faces
            ),
            "body": rospy.Subscriber(
                "/humans/bodies/tracked", IdsList, self._on_tracked_bodies
            ),
            "voice": rospy.Subscriber(
                "/humans/voices/tracked", IdsList, self._on_tracked_voices
            ),
            "tracked_persons": rospy.Subscriber(
                "/humans/persons/tracked", IdsList, self._on_tracked_persons
            ),
            "known_persons": rospy.Subscriber(
                "/humans/persons/known", IdsList, self._on_known_persons
            ),
        }

    def __del__(self):
        self.close()

    def close(self):

        if self._subscribers:
            for _, sub in self._subscribers.items():
                sub.unregister()

            self._subscribers = {}

            rospy.logdebug("HRI listener closed")

    def _update_trackers(self, tracker, tracker_class, new_ids):

        new_ids = set(new_ids)
        current_ids = set(tracker.keys())

        to_remove = current_ids - new_ids
        to_add = new_ids - current_ids

        for id in to_remove:
            tracker[id].close()
            del tracker[id]

        for id in to_add:
            tracker[id] = tracker_class(id, self._tf_buffer, self._reference_frame)

            # Person's instance need access to the list of detect faces/bodies/voices
            # to return the right one
            if tracker_class == Person:
                tracker[id].faces_ = self.faces
                tracker[id].bodies_ = self.bodies
                tracker[id].voices_ = self.voices

    def _on_tracked_faces(self, tracked: IdsList):

        self._update_trackers(self.faces, Face, tracked.ids)

    def _on_tracked_bodies(self, tracked: IdsList):

        self._update_trackers(self.bodies, Body, tracked.ids)

    def _on_tracked_voices(self, tracked: IdsList):

        self._update_trackers(self.voices, Voice, tracked.ids)

    def _on_tracked_persons(self, tracked: IdsList):

        self._update_trackers(self.tracked_persons, Person, tracked.ids)

    def _on_known_persons(self, tracked: IdsList):

        self._update_trackers(self.known_persons, Person, tracked.ids)

    def get_faces(self) -> Mapping[str, Face]:
        """Returns the list of all currently tracked faces, mapped to their ID."""
        return self.faces.copy()

    def on_face(self, callback):
        """Registers a callback function, to be invoked everytime a new face
        is detected."""
        self.face_callbacks.append((callback))

    def on_face_lost(self, callback):
        """Registers a callback function, to be invoked everytime a
        previously tracked face is lost (eg, not detected anymore)
        """
        self.face_lost_callbacks.append((callback))

    def get_bodies(self) -> Mapping[str, Body]:
        """Returns the list of all currently tracked bodies, mapped to their ID."""
        return self.bodies.copy()

    def on_body(self, callback):
        """Registers a callback function, to be invoked everytime a new body
        is detected.
        """
        self.body_callbacks.append((callback))

    def on_body_lost(self, callback):
        """Registers a callback function, to be invoked everytime a
        previously tracked body is lost (eg, not detected anymore)
        """
        self.body_lost_callbacks.append((callback))

    def get_voices(self) -> Mapping[str, Voice]:
        """Returns the list of all currently 'tracked' voices, mapped to their ID."""
        return self.voices.copy()

    def on_voice(self, callback):
        """Registers a callback function, to be invoked everytime a new voice
        is detected.
        """
        self.voice_callbacks.append((callback))

    def on_voice_lost(self, callback):
        """Registers a callback function, to be invoked everytime a
        previously tracked voice is lost (eg, not detected anymore)
        """
        self.voice_lost_callbacks.append((callback))

    def get_persons(self) -> Mapping[str, Person]:
        """Returns the list of all known persons, whether or not they are
        currently actively detected (eg, seen). The persons are mapped to their
        IDs.

        While person do *not* disappear in general, *anonymous* persons (created
        because, eg, a face has been detected, and we can infer a
        yet-to-be-recognised person does exist) can disappear.  Use the `valid()`
        method to ensure the person still exists.
        """
        return self.known_persons.copy()

    def on_person(self, callback):
        """Registers a callback function, to be invoked everytime a new person
        is detected.
        """
        self.known_person_callbacks.append((callback))

    def on_person_lost(self, callback):
        """Registers a callback function, to be invoked everytime a person
        is lost. This can *only* happen for anonymous persons. Identified persons
        will never be removed from the list of all known persons.
        """
        self.known_person_lost_callbacks.append((callback))

    def get_tracked_persons(self) -> Mapping[str, Person]:
        """Returns the list of currently detected persons, mapped to their IDs

        Note that, while person do *not* disappear in general, *anonymous*
        persons (created because, eg, a face has been detected, and we can infer
        a yet-to-be-recognised person does exist) can disappear.
        Use the `valid()` method to ensure the person still exists.
        """
        return self.tracked_persons.copy()

    def on_tracked_person(self, callback):
        """Registers a callback function, to be invoked everytime a new person
        is detected and actively tracked (eg, currently seen).
        """
        self.person_tracked_callbacks.append((callback))

    def on_tracked_person_lost(self, callback):
        """Registers a callback function, to be invoked everytime a previously tracked
        person is lost.
        """
        self.person_tracked_lost_callbacks.append((callback))

    def set_reference_frame(frame):
        """Sets the reference frame from which the TF transformations of the persons will
        be returned (via `Person::transform()`).

        By default, `base_link`.
        """
        self._reference_frame = frame
