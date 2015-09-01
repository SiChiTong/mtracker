## MTracker package for ROS 

#### The package was written for ROS version Hydromedusa. It contains several nodes which serve as an interface for MTracker users. MTracker is a two-wheeled miniature mobile robot designed and produced at Chair of Control and Systems Engineering of Poznan University of Technology, Poland. 

#### The current nodes include:
* The *mtracker* node -- the main driver of the robot which collects the control signals (from the /controls topic) and delivers them to the low-level controller but also reads the odometry information from the robot and publishes it under topics /position and /velocity.
* The *manual_controller* node - a simple node that translates the general joystick deflections into control signals and publish the under topic /controls.
* The *automatic_controller* node - a closed loop controller for the robot. It subscribes to messages from topics: /position, /velocity as well as /reference_position and /reference_velocity. As a result it outputs control signals under topic /controls.
* The *reference_generator* node - a time-based trajectory generator used by the automatic_controller node. It publishes messages under topics: /reference_position and /reference_velocity.
* The *simulator* node - a simple node that simulates the physical robot and helps with offline working. It subscribes to messages from /controls topic and publishes the virtual odometry information under topics /position and /velocity.

*Mateusz Przybyla*