import rospy


class Voice:
    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/voices/" + id

        rospy.logdebug("New voice detected: " + self.ns)

    def close(self):
        pass

    def __str__(self):
        return self.id
