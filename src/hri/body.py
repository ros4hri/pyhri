import rospy
from sensor_msgs.msg import RegionOfInterest, Image


class Body:
    def __init__(self, id):
        self.id = id
        self.ns = "/humans/bodies/" + id

        rospy.logdebug("New body detected: " + self.ns)

        self.roi_sub = rospy.Subscriber(self.ns + "/roi", RegionOfInterest, self.on_roi)

        self.cropped_sub = rospy.Subscriber(
            self.ns + "/cropped", Image, self.on_cropped
        )

    def on_roi(self, msg):
        pass

    def on_cropped(self, msg):
        pass
