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

from .body import Body
from .face import Face
from .voice import Voice
from .person import Person
from hri_msgs.msg import IdsList

from typing import Mapping


class HRIListener:
    def __init__(self, nh):
        self.node_ = nh

        self.feature_subscribers_ = {}

        self.faces = {}
        self.face_callbacks = []
        self.face_lost_callbacks = []

        self.bodies = {}
        self.body_callbacks = []
        self.body_lost_callbacks = []

        self.voices = {}
        self.voice_callbacks = []
        self.voice_lost_callbacks = []

        self.persons = {}
        self.person_callbacks = []
        self.person_lost_callbacks = []
        self.tracked_persons = {}
        self.person_tracked_callbacks = []
        self.person_tracked_lost_callbacks = []

        self._tf_buffer = None
        self._tf_listener = None

    def init(self):
        pass

    def onTrackedFeature(self, feature_type, tracked: IdsList):
        pass

    def getFaces(self) -> Mapping[str, Face]:
        pass

    # Registers a callback function, to be invoked everytime a new face
    # is detected.
    # /
    def onFace(self, callback):
        self.face_callbacks.append((callback))

    # Registers a callback function, to be invoked everytime a
    # previously tracked face is lost (eg, not detected anymore)
    # /
    def onFaceLost(self, callback):
        self.face_lost_callbacks.append((callback))

    # Returns the list of currently detected bodies, mapped to their IDs
    #
    # Bodies are returned as constant std::weak_ptr as they may disappear at any point.
    # /
    def getBodies(self) -> Mapping[str, Body]:
        pass

    # Registers a callback function, to be invoked everytime a new body
    # is detected.
    # /
    def onBody(self, callback):
        self.body_callbacks.append((callback))

    # Registers a callback function, to be invoked everytime a
    # previously tracked body is lost (eg, not detected anymore)
    # /
    def onBodyLost(self, callback):
        self.body_lost_callbacks.append((callback))

    # Returns the list of currently detected voices, mapped to their IDs
    #
    # Voices are returned as constant std::weak_ptr as they may disappear at any point.
    # /
    def getVoices(self) -> Mapping[str, Voice]:
        pass

    # Registers a callback function, to be invoked everytime a new voice
    # is detected.
    # /
    def onVoice(self, callback):
        self.voice_callbacks.append((callback))

    # Registers a callback function, to be invoked everytime a
    # previously tracked voice is lost (eg, not detected anymore)
    # /
    def onVoiceLost(self, callback):
        self.voice_lost_callbacks.append((callback))

    # Returns the list of all known persons, whether or not they are
    # currently actively detected (eg, seen). The persons are mapped to their
    # IDs.
    #
    # Persons are returned as constant std::weak_ptr: while person do *not*
    # disappear in general, *anonymous* persons (created because, eg, a face has
    # been detected, and we can infer a yet-to-be-recognised person does exist)
    # can disappear.
    # /
    def getPersons(self) -> Mapping[str, Person]:
        pass

    # Registers a callback function, to be invoked everytime a new person
    # is detected.
    # /
    def onPerson(self, callback):
        self.person_callbacks.append((callback))

    # Registers a callback function, to be invoked everytime a person
    # is lost. This can *only* happen for anonymous persons. Identified persons
    # will never be removed from the list of all known persons.
    # /
    def onPersonLost(self, callback):
        self.person_lost_callbacks.append((callback))

    # Returns the list of currently detected persons, mapped to their IDs
    #
    # Persons are returned as constant std::weak_ptr: while person do *not* disappear in
    # general, *anonymous* persons (created because, eg, a face has been detected, and we
    # can infer a yet-to-be-recognised person does exist) can disappear.
    # /
    def getTrackedPersons(self) -> Mapping[str, Person]:
        pass

    # Registers a callback function, to be invoked everytime a new person
    # is detected and actively tracked (eg, currently seen).
    # /
    def onTrackedPerson(self, callback):
        self.person_tracked_callbacks.append((callback))

    # Registers a callback function, to be invoked everytime a previously tracked
    # person is lost.
    # /
    def onTrackedPersonLost(self, callback):
        self.person_tracked_lost_callbacks.append((callback))

    # sets the reference frame from which the TF transformations of the persons will
    # be returned (via `Person::transform()`).
    #
    # By default, `base_link`.
    # /
    def setReferenceFrame(frame):
        self._reference_frame = frame
