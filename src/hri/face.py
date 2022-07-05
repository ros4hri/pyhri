import rospy
from sensor_msgs.msg import RegionOfInterest, Image
from hri_msgs.msg import FacialLandmarks, SoftBiometrics


class Face:
    def __init__(self, id):
        self.id = id
        self.ns = "/humans/faces/" + id

        rospy.logdebug("New face detected: " + self.ns)

        self.roi_sub = rospy.Subscriber(self.ns + "/roi", RegionOfInterest, self.on_roi)

        self.cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self.on_cropped
        )

        self.aligned_sub = rospy.Subscriber(
            self.ns + "/aligned", Image, self.on_aligned
        )

        self.landmarks_sub = rospy.Subscriber(
            self.ns + "/landmarks", FacialLandmarks, self.on_landmarks
        )

        self.softbiometrics_sub = rospy.Subscriber(
            self.ns + "/softbiometrics", SoftBiometrics, self.on_softbiometrics
        )

    def on_roi(self, msg):
        pass

    def on_cropped(self, msg):
        pass

    def on_aligned(self, msg):
        pass

    def on_landmarks(self, msg):
        pass

    def on_softbiometrics(self, msg):
        pass
