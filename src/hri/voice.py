import rospy


class Voice:
    def __init__(self, id, tf_buffer, reference_frame):
        self.id = id
        self.ns = "/humans/voices/" + id

        self._valid = True

        rospy.logdebug("New voice detected: " + self.ns)

    def close(self):
        pass

    def valid(self) -> bool:
        """Returns True if this voice still exists (and thus is valid).
        If False, methods like `Voice.transform` will raise an exception.
        """
        return self._valid

    def transform(self):
        raise NotImplementedError("method not yet implemented")

    def __str__(self):
        return self.id
