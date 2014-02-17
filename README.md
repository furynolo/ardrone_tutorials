This is my version of the forked repository with image processing imlemented so the ardrone can react to objects identified in the images.
It also has logic for locating an object and moving towards it.


First you need to turn on the ardrone and then connect your laptop to the ardrone's network
The first thing that should be run is src/keyboard_controller.py
(this allows keyboard commands to be be sent to the drone to controll it).
Running src/decision_controller.py sends commands to the ardrone to control its movement.

Debugging:
Running src/sub_decision.py allows you to see what decision commands are being published and sent to the drone.
Running src/pub_decision.py allows you to send send individual commands to the ardrone.


From the original README:
Up and flying with the AR.Drone and ROS
========================================================

This repository contains the source-code for the Up and flying with the AR.Drone and ROS tutorial series, published on [Robohub](http://www.robohub.org).

A stable branch is maintained for each of the released tutorials, with the master branch always synchronized with the stable branch of the latest tutorial. Each tutorial's branch also contains all source-code required for the previous tutorials.

Virtual Machine
---------------

These tutorials are (optionally) supported by the custom [ARDroneUbuntu virtual machine](http://bit.ly/VqtDql). This includes all software and source-code required to run this tutorial. Refer to the first tutorial on how to use the virtual machine.
