# MTracker package for ROS 

### The package was written for ROS version Hydromedusa. It contains several nodes which serve as an interface for MTracker users. MTracker is a two-wheeled miniature mobile robot designed and produced at Chair of Control and Systems Engineering of Poznan University of Technology, Poland. 

### The current nodes are: ###

* mtracker - main driver of the robot which connects to the low-level controller and sets the vehicles velocities; publishes actual position and velocity of the robot
* mtracker_joy - translates the general joystick deflections to control signals
* controller - a closed loop controller for the robot
* simulator - simulates the physical robot and helps working offline, without the actual robot