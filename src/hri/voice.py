import rospy


class Voice:
    def __init__(self, id):
        self.id = id
        self.ns = "/humans/voices/" + id

        rospy.logdebug("New voice detected: " + self.ns)
