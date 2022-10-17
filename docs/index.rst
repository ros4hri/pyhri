pyhri
=====

`pyhri` is a Python wrapper for the ROS4HRI framework. It implements the `ROS
REP-155 <https://www.ros.org/reps/rep-0155.html>`_.

Example:

.. code:: python

  import rospy
  from hri import HRIListener
  
  rospy.init_node("pyhri_test")
  
  hri = HRIListener()
  
  # (start a ROS4HRI-compatible face detector like hri_face_detect)
  
  while not rospy.is_shutdown():
    # access the detected faces:
    for id, face in hri.faces.items():
        print("Currently seeing face %s" % id)
    
    # (start a ROS4HRI-compatible person identification pipeline)
    # (for instance, hri_face_identification and hri_person_manager)
    
    # access known people:
    for id, person in hri.tracked_persons.items():
        if person.face:
            print("Person %s is bound to face %s" % (id, person.face.id))
            print("6D gaze transform: %s" % (id, person.face.gaze_transform()))
    
    
    rospy.sleep(1)

.. toctree::
   :maxdepth: 2
   :caption: Contents:

API
===

.. autosummary::
   :toctree: generated
   :recursive:

   hri.hri
   hri.person
   hri.face
   hri.body
   hri.voice

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
