^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyhri
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2022-10-18)
------------------
* autogenerate docs with rosdoc (for display on ROS wiki) and SPHINX
* More api documentation
* adjust code base for ReadTheDocs doc generation
  In particular:
  - add a requirements.txt file for catkin
  - allows importing pyhri without rospy
* Face.transform|Face.gaze_transform can change their reference frame
* removed dep on numpy.typing as it is not widely available yet
* mark features as invalid once they disappear
* returns copy of persons to avoid modifying dictionary while iterating
  While here, add infrastrcutre to check whether a face/body/voice/person is still valid
* expose the tf transform of the face and gaze
* add BSD license file
* [test] further tuning of tests' waiting behaviours for #1
* Contributors: Séverin Lemaignan

0.1.3 (2022-07-13)
------------------
* refining waiting + closing in tests -- they all pass reliably on my machine
* unregister from main topics when closing HRIListener
* Contributors: Séverin Lemaignan

0.1.2 (2022-07-12)
------------------
* [cmake] add test only behing CATKIN_ENABLE_TESTING
* Contributors: Séverin Lemaignan

0.1.1 (2022-07-06)
------------------
* various fixes after first test with actual ROS4HRI messages
* Contributors: Séverin Lemaignan

0.1.0 (2022-07-06)
------------------
* initial implementation, closely following the libhri C++ API
* all unit-tests pass (modulo random timing/waiting issues)
* add basic README
* Contributors: Séverin Lemaignan
