[![Documentation Status](https://readthedocs.org/projects/pyhri/badge/?version=latest)](https://pyhri.readthedocs.io/en/latest/?badge=latest)

pyhri: Python wrapper for the ROS4HRI framework
===============================================

Documentation
-------------

API Documentation: https://pyhri.readthedocs.io

Example usage
-------------

```python
import rospy
from hri import HRIListener

rospy.init_node("pyhri_test")

hri = HRIListener()

# (start a ROS4HRI-compatible face detector like hri_face_detect)

# access the detected faces:
for id, face in hri.faces.items():
    print("Currently seeing face %s" % id)

# (start a ROS4HRI-compatible person identification pipeline)
# (for instance, hri_face_identification and hri_person_manager)

# access known people:
for id, person in hri.tracked_persons.items():
    print("Person %s is bound to face %s" % (id, person.face.id))


# rospy.spin()

```

