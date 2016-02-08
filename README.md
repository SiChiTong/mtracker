## The mtracker package

The mtracker package contains several nodes which serve as an interface for MTracker users. MTracker is a two-wheeled miniature mobile robot designed and produced at Chair of Control and Systems Engineering of Poznan University of Technology. The package also contains several launch files designed to start various scenarios (e.g. real robot with manual control, simulated robot with automatic control).

#### The nodes

The nodes are supposed to work on the on-board computer of MTracker. They use standard ros messages, e.g. `geometry_msgs/Pose2D` for pose and `geometry_msg/Twist` for velocities and control signals. The names of the topics are configured by the global parameters (see below). The package contains following nodes: 

* `mtracker` - the main driver of the robot which collects the control signals and delivers them to the low-level controller via serial port. It also reads the odometric information and publishes the robot pose and velocity. 
* `simulator` - a simple simulator that can be used simultaneously with or as a substitute for the real robot. It helps while working 'offline'. It subscribes to control messages and publishes the virtual odometry information.
* `manual_controller` - a simple node that translates the general joystick or keyboard (type `geometry_msgs/Twist`) messages into control signals. 
* `automatic_controller` - a closed loop controller that collects actual reference (desired) and measured pose and velocity and publishes the control signals.
* `reference_generator` - a time-based trajectory generator used by the `automatic_controller` node. It publishes the reference pose and velocity for the robot.
* `controls_scaling` - a simple node that limits the control signals. The algorithm scales the values control signals so that the control vector preserves its orientation.

#### The launch files

In order to simplify the process of starting off the nodes, several launch files have been prepared. Each of them starts the RViz with preconfigured environment. Launch files that use the real MTracker robot include the network configurations that may vary depending on the actual robot and network. In the mentioned launch files the roscore works on-board the robot. The package containts the following launch files:

* `mtracker_automatic` - starts the mtracker with the automatic controller. 
* `mtracker_manual` - starts the mtracker with the manual controller.
* `simulator_automatic` - starts the simulator with the automatic controller.
* `simulator_manual` - starts the simulator with the manual controller.

#### The global parameters

The nodes use a collection of global parameter which help to keep the topics name consistent. The global parameters are (square brackets enclose default value):

* `loop_rate` [100] - integer value of main loop frequency in hertz (some of the nodes work in an asynchronous manner).
* `controls_topic` [controls] - topic of control signals.
* `scaled_controls_topic` [scaled_controls] - topic of control signals after scaling procedure (published by controls_scaling node).
* `pose_topic` [pose] - topic of pose meassages used for automatic controller. The stream of messages can be obtained from external measuring tool or an observer.
* `odom_pose_topic` [odom_pose] - topic of odometry-based pose published by mtracker. It can be renamed to 'pose' in the case of no external measuring tool.
* `virtual_pose_topic` [virtual_pose] - topic of pose obtained from simulated robot.
* `reference_pose_topic` [reference_pose] - topic of pose obtained from reference generator.
* `velocity_topic` [velocity] - topic of velocity meassages used for automatic controller. The stream of messages can be obtained from external measuring tool or an observer.
* `odom_velocity_topic` [odom_velocity] - topic of odometry-based velocity published by mtracker. It can be renamed to 'velocity' in the case of no external measuring tool.
* `virtual_velocity_topic` [virtual_velocity] - topic of velocity obtained from simulated robot.
* `reference_velocity_topic` [reference_velocity] - topic of velocity obtained from reference generator.
* `joy_topic` [joy] - topic of joystick messages used by manual controller.
* `keys_topic` [keys] - topic of keyboard messages used by manual controller.

#### The local parameters
Some of the nodes use local parameters that can define their behavior. The local parameters are (square brackets enclose default value):

* `simulator` node:
    * `time_constant` [0.1] - an artificial inertia of the robot. Results in an smooth changes in velocity.
* `manual_controller` node:
    * `linear_gain` [0.4] - proportion between linear velocity (control signal) and forward deflection of joystick (default value and joystick deflected forward to maximum value will result in robot moving forward with 0.4 m/s speed).
    * `angular_gain` [1.5] - proportion between angular velocity (control signal) and sideways deflection of joystick (default value and joystick deflected sideways to maximum value will result in robot rotating with 1.5 rad/s speed).
* `reference_generator` node:
    * `trajectory_type` [0] - one of predefined types of reference trajectories:
        * 0 - point-like trajectory,
        * 1 - linear trajectory,
        * 2 - harmonic trajectory,
        * 3 - lemniscate trajectory.
    * `trajectory_paused` [false] - if set to true, the reference generator is paused and need to be triggered. If set to false, the reference generator publishes information immediately after start.
* `controls_scaling` node:
    * `max_wheel_rate` [6.0] - maximal allowable angular velocity of each wheel of the robot in rad/s.