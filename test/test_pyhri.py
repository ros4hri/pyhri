#!/usr/bin/env python

PKG = "pyhri"
NAME = "test_pyhri"

import sys
import unittest

import rospy
import rostest

from sensor_msgs.msg import RegionOfInterest
from hri_msgs.msg import IdsList

import hri


class TestHRI(unittest.TestCase):
    def __init__(self, *args):
        super(TestHRI, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        self.faces_pub = rospy.Publisher(
            "/humans/faces/tracked", IdsList, queue_size=1, latch=False
        )

    def wait(self):
        rospy.sleep(0.03)

    def test_get_faces(self):

        hri_listener = hri.HRIListener()

        self.wait()

        self.assertEquals(self.faces_pub.get_num_connections(), 1)
        self.assertEquals(len(hri_listener.get_faces()), 0)

        self.faces_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.get_faces()), 1)
        self.assertIn("A", hri_listener.get_faces())

        self.faces_pub.publish(ids=["A"])
        self.wait()
        self.assertEquals(len(hri_listener.get_faces()), 1)

        self.faces_pub.publish(ids=["A", "B"])
        self.wait()
        self.assertEquals(len(hri_listener.get_faces()), 2)
        self.assertIn("A", hri_listener.get_faces())
        self.assertIn("B", hri_listener.get_faces())

        self.faces_pub.publish(ids=[])
        self.wait()
        self.assertEquals(len(hri_listener.get_faces()), 0)

    def test_get_faces_roi(self):

        hri_listener = hri.HRIListener()

        roi_A_pub = rospy.Publisher(
            "/humans/faces/A/roi", RegionOfInterest, queue_size=1, latch=True
        )
        roi_B_pub = rospy.Publisher(
            "/humans/faces/A/roi", RegionOfInterest, queue_size=1, latch=True
        )

        self.faces_pub.publish(ids=["A"])
        self.wait()

        self.assertEquals(roi_A_pub.get_num_connections(), 1)


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestHRI, sys.argv)
