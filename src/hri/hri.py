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

        self._faces: Mapping[str, Face] = {}
        self.face_callbacks = []
        self.face_lost_callbacks = []

        self._bodies: Mapping[str, Body] = {}

        self.body_callbacks = []
        self.body_lost_callbacks = []

        self._voices: Mapping[str, Voice] = {}

        self.voice_callbacks = []
        self.voice_lost_callbacks = []

        self._tracked_persons: Mapping[str, Person] = {}

        self.person_tracked_callbacks = []
        self.person_tracked_lost_callbacks = []

        self._known_persons: Mapping[str, Person] = {}

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

        new_cbs = []
        lost_cbs = []

        if tracker_class == Face:
            new_cbs = self.face_callbacks
            lost_cbs = self.face_lost_callbacks

        elif tracker_class == Body:
            new_cbs = self.body_callbacks
            lost_cbs = self.body_lost_callbacks

        elif tracker_class == Voice:
            new_cbs = self.voice_callbacks
            lost_cbs = self.voice_lost_callbacks

        elif tracker == self._tracked_persons:
            new_cbs = self.person_tracked_callbacks
            lost_cbs = self.person_tracked_lost_callbacks

        elif tracker == self._known_persons:
            new_cbs = self.known_person_callbacks
            lost_cbs = self.known_person_lost_callbacks

        for id in to_remove:
            for cb in lost_cbs:
                cb(id)
            tracker[id].close()
            del tracker[id]

        for id in to_add:
            tracker[id] = tracker_class(id, self._tf_buffer, self._reference_frame)

            for cb in new_cbs:
                cb(tracker[id])

            # Person's instance need access to the list of detect faces/bodies/voices
            # to return the right one
            if tracker_class == Person:
                tracker[id]._hrilistener = self

    def _on_tracked_faces(self, tracked: IdsList):

        self._update_trackers(self._faces, Face, tracked.ids)

    def _on_tracked_bodies(self, tracked: IdsList):

        self._update_trackers(self._bodies, Body, tracked.ids)

    def _on_tracked_voices(self, tracked: IdsList):

        self._update_trackers(self._voices, Voice, tracked.ids)

    def _on_tracked_persons(self, tracked: IdsList):

        self._update_trackers(self._tracked_persons, Person, tracked.ids)

    def _on_known_persons(self, tracked: IdsList):

        self._update_trackers(self._known_persons, Person, tracked.ids)

    @property
    def faces(self) -> Mapping[str, Face]:
        """Returns the list of all currently tracked faces, mapped to their ID.

        As faces can 'disappear' at any time (eg if they not detected anymore), always check the
        `valid` attribute of the returned `Face` instances before accessing them.
        """
        return self._faces.copy()

    def on_face(self, callback):
        """Registers a callback function, to be invoked everytime a new face
        is detected.

        The callback must accept one single parameter, the new `Face` instance.
        """
        self.face_callbacks.append((callback))

    def on_face_lost(self, callback):
        """Registers a callback function, to be invoked everytime a
        previously tracked face is lost (eg, not detected anymore).

        The callback must accept one single parameter, the id of the lost face.
        """
        self.face_lost_callbacks.append((callback))

    @property
    def bodies(self) -> Mapping[str, Body]:
        """Returns the list of all currently tracked bodies, mapped to their ID.

        As bodies can 'disappear' at any time (eg if they not detected anymore), always check the
        `valid` attribute of the returned `Body` instances before accessing them.
        """
        return self._bodies.copy()

    def on_body(self, callback):
        """Registers a callback function, to be invoked everytime a new body
        is detected.

        The callback must accept one single parameter, the new `Body` instance.
        """
        self.body_callbacks.append((callback))

    def on_body_lost(self, callback):
        """Registers a callback function, to be invoked everytime a
        previously tracked body is lost (eg, not detected anymore)

        The callback must accept one single parameter, the id of the lost body.
        """
        self.body_lost_callbacks.append((callback))

    @property
    def voices(self) -> Mapping[str, Voice]:
        """Returns the list of all currently 'tracked' voices, mapped to their ID.

        As voices can 'disappear' at any time (eg if they not heard anymore), always check the
        `valid` attribute of the returned `Voice` instances before accessing them.
        """
        return self._voices.copy()

    def on_voice(self, callback):
        """Registers a callback function, to be invoked everytime a new voice
        is detected.

        The callback must accept one single parameter, the new `Voice` instance.
        """
        self.voice_callbacks.append((callback))

    def on_voice_lost(self, callback):
        """Registers a callback function, to be invoked everytime a
        previously tracked voice is lost (eg, not detected anymore)

        The callback must accept one single parameter, the id of the lost body.
        """
        self.voice_lost_callbacks.append((callback))

    @property
    def known_persons(self) -> Mapping[str, Person]:
        """Returns the list of all known persons, whether or not they are
        currently actively detected (eg, seen). The persons are mapped to their
        IDs.

        While person do *not* disappear in general, *anonymous* persons (created
        because, eg, a face has been detected, and we can infer a
        yet-to-be-recognised person does exist) can disappear.  Check the `valid`
        attribute to ensure the person still exists.
        """
        return self._known_persons.copy()

    def on_person(self, callback):
        """Registers a callback function, to be invoked everytime a new person
        is detected.

        The callback must accept one single parameter, the new `Person` instance.
        """
        self.known_person_callbacks.append((callback))

    def on_person_lost(self, callback):
        """Registers a callback function, to be invoked everytime a person
        is lost. This can *only* happen for anonymous persons. Identified persons
        will never be removed from the list of all known persons.

        The callback must accept one single parameter, the id of the lost person.
        """
        self.known_person_lost_callbacks.append((callback))

    @property
    def tracked_persons(self) -> Mapping[str, Person]:
        """Returns the list of currently detected persons, mapped to their IDs

        Note that, while person do *not* disappear in general, *anonymous*
        persons (created because, eg, a face has been detected, and we can infer
        a yet-to-be-recognised person does exist) can disappear.
        Check the `valid` attribute to ensure the person still exists.
        """
        return self._tracked_persons.copy()

    def on_tracked_person(self, callback):
        """Registers a callback function, to be invoked everytime a new person
        is detected and actively tracked (eg, currently seen).

        The callback must accept one single parameter, the new `Person` instance.
        """
        self.person_tracked_callbacks.append((callback))

    def on_tracked_person_lost(self, callback):
        """Registers a callback function, to be invoked everytime a previously tracked
        person is lost.

        The callback must accept one single parameter, the id of the lost person.
        """
        self.person_tracked_lost_callbacks.append((callback))

    def set_reference_frame(self, frame):
        """Sets the reference frame from which the TF transformations of the persons will
        be returned (via `Person::transform()`).

        By default, `base_link`.
        """
        self._reference_frame = frame
