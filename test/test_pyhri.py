#!/usr/bin/env python

PKG = "pyhri"
NAME = "test_pyhri"

import sys
import unittest

import rospy
import rostest

from hri_msgs.msg import IdsList, LiveSpeech, NormalizedRegionOfInterest2D
from std_msgs.msg import String, Bool

import pyhri


class TestHRI(unittest.TestCase):
    def __init__(self, *args):
        super(TestHRI, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)

        self.cb_triggered = False

        self.faces_pub = rospy.Publisher(
            "/humans/faces/tracked", IdsList, queue_size=1, latch=False
        )

        self.bodies_pub = rospy.Publisher(
            "/humans/bodies/tracked", IdsList, queue_size=1, latch=False
        )

        self.voices_pub = rospy.Publisher(
            "/humans/voices/tracked", IdsList, queue_size=1, latch=False
        )

        self.known_persons_pub = rospy.Publisher(
            "/humans/persons/known", IdsList, queue_size=1, latch=False
        )

        self.tracked_persons_pub = rospy.Publisher(
            "/humans/persons/tracked", IdsList, queue_size=1, latch=False
        )
        self.wait()

    def wait(self, delay=0.05):
        rospy.sleep(delay)

    def test_get_faces(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        self.assertEquals(self.faces_pub.get_num_connections(), 1)
        self.assertEquals(len(hri_listener.faces), 0)

        self.faces_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.faces), 1)
        self.assertIn("A", hri_listener.faces)

        self.faces_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.faces), 1)

        self.faces_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.faces), 2)
        self.assertIn("A", hri_listener.faces)
        self.assertIn("B", hri_listener.faces)

        self.faces_pub.publish(ids=[])
        self.wait()
        self.assertEquals(len(hri_listener.faces), 0)

        hri_listener.close()

    def test_get_faces_roi(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        roi_A_pub = rospy.Publisher(
            "/humans/faces/A/roi",
            NormalizedRegionOfInterest2D,
            queue_size=1,
            latch=True,
        )
        roi_B_pub = rospy.Publisher(
            "/humans/faces/B/roi",
            NormalizedRegionOfInterest2D,
            queue_size=1,
            latch=True,
        )

        self.faces_pub.publish(ids=["A"])
        self.wait(delay=0.1)

        self.assertEquals(roi_A_pub.get_num_connections(), 1)

        self.faces_pub.publish(ids=["B"])
        self.wait(delay=0.1)

        self.assertEquals(
            roi_A_pub.get_num_connections(),
            0,
            "Face A is deleted. No one should be subscribed to /humans/faces/A/roi anymore",
        )
        self.assertEquals(
            roi_B_pub.get_num_connections(),
            1,
            "Face B should have subscribed to /humans/faces/B/roi",
        )

        self.assertIsNone(hri_listener.faces["B"].roi)

        roi_B_pub.publish(xmax=0.1)
        self.wait()
        self.assertIsNotNone(hri_listener.faces["B"].roi)
        self.assertAlmostEqual(hri_listener.faces["B"].roi.xmax, 0.1)

        roi_B_pub.publish(xmax=0.2)
        self.wait()
        self.assertAlmostEqual(hri_listener.faces["B"].roi.xmax, 0.2)

        # RoI of face A published *before* face A is published in /faces/tracked,
        # but should still get its RoI, as /roi is latched.
        roi_A_pub.publish(xmax=0.2)
        self.faces_pub.publish(ids=["A", "B"])
        self.wait(delay=0.1)

        self.assertEqual(hri_listener.faces["A"].ns, "/humans/faces/A")
        self.assertAlmostEqual(hri_listener.faces["A"].roi.xmax, 0.2)
        self.assertEqual(hri_listener.faces["B"].ns, "/humans/faces/B")
        self.assertAlmostEqual(hri_listener.faces["A"].roi.xmax, 0.2)

        hri_listener.close()

    def test_get_bodies(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        self.assertEquals(self.bodies_pub.get_num_connections(), 1)
        self.assertEquals(len(hri_listener.bodies), 0)

        self.bodies_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.bodies), 1)
        self.assertIn("A", hri_listener.bodies)
        self.assertEquals(hri_listener.bodies["A"].id, "A")

        self.bodies_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.bodies), 1)

        self.bodies_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.bodies), 2)
        self.assertIn("A", hri_listener.bodies)
        self.assertIn("B", hri_listener.bodies)

        self.bodies_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.bodies), 2)

        self.bodies_pub.publish(ids=["B"])
        self.wait()
        self.assertEquals(len(hri_listener.bodies), 1)
        self.assertIn("B", hri_listener.bodies)
        self.assertEquals(hri_listener.bodies["B"].id, "B")

        self.bodies_pub.publish(ids=[])
        self.wait()
        self.assertEquals(len(hri_listener.bodies), 0)

        hri_listener.close()

    def test_get_voices(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        self.assertEquals(self.voices_pub.get_num_connections(), 1)
        self.assertEquals(len(hri_listener.voices), 0)

        self.voices_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.voices), 1)
        self.assertIn("A", hri_listener.voices)
        self.assertEquals(hri_listener.voices["A"].id, "A")

        self.voices_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.voices), 1)

        self.voices_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.voices), 2)
        self.assertIn("A", hri_listener.voices)
        self.assertIn("B", hri_listener.voices)

        self.voices_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.voices), 2)

        self.voices_pub.publish(ids=["B"])
        self.wait()
        self.assertEquals(len(hri_listener.voices), 1)
        self.assertIn("B", hri_listener.voices)
        self.assertEquals(hri_listener.voices["B"].id, "B")

        self.voices_pub.publish(ids=[])
        self.wait()
        self.assertEquals(len(hri_listener.voices), 0)

        hri_listener.close()

    def cb(self, msg):
        self.cb_triggered = True

    def voice_cb(self, voice):
        self.cb_triggered = True
        voice.on_speaking(self.cb)
        voice.on_incremental_speech(self.cb)
        voice.on_speech(self.cb)

    def test_voice_callbacks(self):

        hri_listener = pyhri.HRIListener()
        
        self.voice_A_is_speaking_pub = rospy.Publisher(
            "/humans/voices/A/is_speaking", Bool, queue_size=1, latch=False
        )

        self.voice_A_speech_pub = rospy.Publisher(
            "/humans/voices/A/speech", LiveSpeech, queue_size=1, latch=False
        )
        self.wait()

        hri_listener.on_voice(self.voice_cb)

        self.cb_triggered = False
        self.voices_pub.publish(ids=["A"])
        self.wait()
        self.assertTrue(self.cb_triggered)
        self.cb_triggered = False

        self.voice_A_is_speaking_pub.publish(data=True)
        self.wait()
        self.assertTrue(self.cb_triggered)
        self.assertTrue(hri_listener.voices["A"].is_speaking)
        self.cb_triggered = False

        self.voice_A_is_speaking_pub.publish(data=False)
        self.wait()
        self.assertTrue(self.cb_triggered)
        self.assertFalse(hri_listener.voices["A"].is_speaking)
        self.cb_triggered = False

        self.voice_A_speech_pub.publish(final="test speech")
        self.wait()
        self.assertTrue(self.cb_triggered)
        self.assertEquals(hri_listener.voices["A"].speech, "test speech")
        self.cb_triggered = False

        self.voice_A_speech_pub.publish(incremental="test speech incremental")
        self.wait()
        self.assertTrue(self.cb_triggered)
        self.assertEquals(
            hri_listener.voices["A"].incremental_speech, "test speech incremental"
        )
        self.cb_triggered = False

        hri_listener.close()

    def test_get_known_persons(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        self.assertEquals(self.known_persons_pub.get_num_connections(), 1)
        self.assertEquals(len(hri_listener.known_persons), 0)

        self.known_persons_pub.publish(ids=["A"])
        self.wait(delay=0.1)
        self.assertEquals(len(hri_listener.known_persons), 1)
        self.assertIn("A", hri_listener.known_persons)
        self.assertEquals(hri_listener.known_persons["A"].id, "A")

        self.known_persons_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.known_persons), 1)

        self.known_persons_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.known_persons), 2)
        self.assertIn("A", hri_listener.known_persons)
        self.assertIn("B", hri_listener.known_persons)

        self.known_persons_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.known_persons), 2)

        self.known_persons_pub.publish(ids=["B"])
        self.wait()
        self.assertEquals(len(hri_listener.known_persons), 1)
        self.assertIn("B", hri_listener.known_persons)
        self.assertEquals(hri_listener.known_persons["B"].id, "B")

        self.known_persons_pub.publish(ids=[])
        self.wait()
        self.assertEquals(len(hri_listener.known_persons), 0)

        hri_listener.close()

    def test_get_tracked_persons(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        self.assertEquals(self.tracked_persons_pub.get_num_connections(), 1)
        self.assertEquals(len(hri_listener.tracked_persons), 0)

        self.tracked_persons_pub.publish(ids=["A"])
        self.wait(delay=0.1)
        self.assertEquals(len(hri_listener.tracked_persons), 1)
        self.assertIn("A", hri_listener.tracked_persons)
        self.assertEquals(hri_listener.tracked_persons["A"].id, "A")

        self.tracked_persons_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.tracked_persons), 1)

        self.tracked_persons_pub.publish(ids=["A", "B"])
        self.wait(delay=0.1)
        self.assertEquals(len(hri_listener.tracked_persons), 2)
        self.assertIn("A", hri_listener.tracked_persons)
        self.assertIn("B", hri_listener.tracked_persons)

        self.tracked_persons_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.tracked_persons), 2)

        self.tracked_persons_pub.publish(ids=["B"])
        self.wait(delay=0.1)
        self.assertEquals(len(hri_listener.tracked_persons), 1)
        self.assertIn("B", hri_listener.tracked_persons)
        self.assertEquals(hri_listener.tracked_persons["B"].id, "B")

        self.tracked_persons_pub.publish(ids=[])
        self.wait()
        self.assertEquals(len(hri_listener.tracked_persons), 0)

        hri_listener.close()

    def test_person_attributes(self):

        hri_listener = pyhri.HRIListener()
        self.wait()

        person_face_pub = rospy.Publisher(
            "/humans/persons/p1/face_id", String, queue_size=1, latch=True
        )

        self.tracked_persons_pub.publish(ids=["p1"])
        self.faces_pub.publish(ids=["f1", "f2"])
        self.wait(delay=0.1)

        self.assertIn("p1", hri_listener.tracked_persons)
        p1 = hri_listener.tracked_persons["p1"]

        self.assertFalse(
            p1.anonymous, "by default, persons are not supposed to be anonymous"
        )

        self.assertIsNone(p1.face)

        person_face_pub.publish(data="f1")
        self.wait(delay=0.1)

        self.assertIsNotNone(p1.face)

        self.assertEquals(p1.face.id, "f1")

        hri_listener.close()


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestHRI, sys.argv)
