## The mtracker package

The mtracker package contains several nodes which serve as an interface for MTracker users. MTracker is a two-wheeled miniature mobile robot designed and produced at Chair of Control and Systems Engineering of Poznan University of Technology. For graphical user interface check mtracker_gui package.

This readme is organized as follows:

* [1. The nodes](#markdown-header-1-the-nodes)
* [2. The launch files](#markdown-header-2-the-launch-files)
* [3. The global parameters](#markdown-header-3-the-global-parameters)
* [4. The local parameters](#markdown-header-4-the-local-parameters)
* [5. The scripts](#markdown-header-5-the-scripts)

### 1. The nodes

The nodes are supposed to work on the on-board computer of MTracker. They use standard ros messages, e.g. `geometry_msgs/Pose2D` for pose and `geometry_msg/Twist` for velocities and control signals. The names of the topics are configured by the global parameters (see below). The package contains following nodes: 

* `mtracker` - the main driver of the robot which collects the control signals and delivers them to the low-level controller via serial port. It also reads the odometric information and publishes the robot pose and velocity. 
* `simulator` - a simple simulator that can be used simultaneously with or as a substitute for the real robot. It helps while working 'offline'. It subscribes to control messages and publishes the virtual odometry information.
* `state_estimator` - a collector of all available measurement data and control signals which tries to estimate the best possible pose and velocity of the robot.
* `manual_controller` - a simple node that translates the general joystick or keyboard (type `geometry_msgs/Twist`) messages into control signals. 
* `automatic_controller` - a closed loop controller for trajectory tracking purpose. 
* `obstacle_controller` - a closed loop controller for point-to-point motion with obstacle avoidance. 
* `reference_generator` - a time-based trajectory generator used by the `automatic_controller` node. 
* `controls_scaling` - a limiter of the control signals. The algorithm scales the values of control signals so that the control vector preserves its orientation.
* `data_recorder` - a collector of all available signals which can record them and save to .txt or .yaml file.

### 2. The launch files

In order to simplify the process of starting off the nodes, two example launch files have been prepared. Each of them starts the RViz with preconfigured environment using mtracker_gui. The package contains the following launch files:

* `mtracker` - configures network variables and starts all of the nodes. The roscore is assumed to work on the MTracker on-board computer therefor it must be a'priori started off.
* `simulator` - starts all of the nodes needed for the off-line, simulation-only work.

### 3. The global parameters

The nodes use a collection of global parameters which help to keep the topics names consistent. The global parameters are (square brackets enclose default value):

* `loop_rate` [100] - integer value of main loop frequency in hertz (some of the nodes work in an asynchronous manner).
* `world_frame` [world] - name of the global coordinate frame used by tf.

* `controls_topic` [controls] - topic of control signals published directly by the motion controllers.
* `scaled_controls_topic` [scaled_controls] - topic of control signals after scaling procedure. The mtracker and simulator nodes subscribe to scaled control signals.
* `joy_topic` [joy] - topic of joystick messages used by manual controller.
* `keys_topic` [keys] - topic of keyboard messages used by manual controller.

* `pose_topic` [pose] - topic of meassages describing best estimates of robot pose. 
* `velocity_topic` [velocity] - topic of meassages describing best estimates of robot velocity. 
* `odom_pose_topic` [odom_pose] - topic of odometry-based pose published by mtracker. 
* `odom_velocity_topic` [odom_velocity] - topic of odometry-based velocity published by mtracker. 
* `virtual_pose_topic` [virtual_pose] - topic of pose obtained from simulated robot.
* `virtual_velocity_topic` [virtual_velocity] - topic of velocity obtained from simulated robot.
* `reference_pose_topic` [reference_pose] - topic of pose obtained from reference generator.
* `reference_velocity_topic` [reference_velocity] - topic of velocity obtained from reference generator.

* `optitrack_pose_topic` [optitrack_pose] - topic of pose messages obtained from the `mocap_optitrack` package if used.
* `obstacles_topic` [obstacles] - topic of obstacles messages obtained from the `obstacle_detector` package if used.

### 4. The local parameters

Most of the nodes use additional local parameters that can define their behavior. The local parameters are (square brackets enclose default value):
child_frame

* `mtracker` node:
    * `child_frame` [odometry] - name of the local coordinate frame attached to the pose obtained from the odometry.
* `simulator` node:
    * `time_constant` [0.1] - an artificial inertia of the robot (described in seconds). Results in an smooth changes of velocity.
    * `child_frame` [virtual] - name of the local coordinate frame attached to the pose obtained from the simulator.
    * `initial_x` [0.0] - initial value of x coordinate (in meters) for the simulator.
    * `initial_y` [0.0] - initial value of y coordinate (in meters) for the simulator.
    * `initial_theta` [0.0] - initial value of orientation (in radians) for the simulator.
* `state_estimator` node:
    * `child_frame` [robot] - name of the local coordinate frame attached to the pose obtained from the state estimator.
* `manual_controller` node:
    * `k_v` [0.4] - gain of linear velocity control signal (e.g. if joystick was deflected forward to value 0.5, the controller will result in linear velocity of 0.5 * k_v m/s).
    * `k_w` [1.5] - gain of angular velocity control signal (e.g. if joystick was deflected sideways to value 1.0, the controller will result in angular velocity of 1.0 * k_w rad/s).
* `reference_generator` node:
    * `child_frame` [reference] - name of the local coordinate frame attached to the pose obtained from the reference generator.
    * `trajectory_type` [0] - one of predefined types of reference trajectories:
        * 0 - point-like trajectory,
        * 1 - linear trajectory,
        * 2 - harmonic trajectory,
        * 3 - lemniscate trajectory.
    * `trajectory_paused` [false] - if set to true, the reference generator is paused and need to be triggered. If set to false, the reference generator publishes information immediately after start.
* `controls_scaling` node:
    * `max_wheel_rate` [15.0] - maximal allowable angular velocity of each wheel of the robot in rad/s.
* `data_recorder` node:
    * `use_txt` [true] - if true, save data to .txt file.
    * `use_yaml` [false] - if true, save data to .yaml file.

### 5. The scripts

Two shell scripts have been prepared for updating and starting the mtracker off.

* `mtracker_update.sh` - removes the old version of `mtracker` package from the robot, copies the new one and builds it. It need proper ssh configuration between the computers.
* `mtracker_start.sh` - sets the environment variables both on the robot and on the user's computer, runs roscore and launches the mtracker launch file.

