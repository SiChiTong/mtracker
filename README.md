## MTracker package for ROS 

#### The package was written for ROS version Hydromedusa. It contains several nodes which serve as an interface for MTracker users. MTracker is a two-wheeled miniature mobile robot designed and produced at Chair of Control and Systems Engineering of Poznan University of Technology, Poland. 

#### The current nodes include:
* The `mtracker` node - the main driver of the robot which collects the control signals (from the `/controls` topic) and delivers them to the low-level controller but also reads the odometry information from the robot and publishes it under topics `/pose` and `/velocity`. It also publishes a "robot" tf frame for visualisation purposes.
* The `manual_controller` node - a simple node that translates the general joystick deflections from `/joy` topic into control signals and publish them under topic `/controls`.
* The `automatic_controller` node - a closed loop controller for the robot. It subscribes to messages from topics: `/pose`, `/velocity` as well as `/reference_pose` and `/reference_velocity`. As a result it outputs control signals under topic `/controls`.
* The `reference_generator` node - a time-based trajectory generator used by the `automatic_controller` node. It publishes messages under topics: `/reference_position` and `/reference_velocity`. It also publishes a "reference" tf frame for visualisation purposes.
* The `simulator` node - a simple node that simulates the physical robot and helps while working offline. It subscribes to messages from `/controls` topic and publishes the virtual odometry information under topics `/pose` and `/velocity`. It also publishes a "virtual_robot" tf frame.

Author:
*Mateusz Przybyla*