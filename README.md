# Project: Capstone

### Udacity Self-Driving Car Engineer Nanodegree Program

---

This is the final project for the nanodegree. You are given a set of (already working) ROS nodes, where you make minor modifications to make the system work with the provided simulator and (possibly with some extensions) a real self-driving car called CARLA.

[//]: # (Image References)
[arch]: ./imgs/architecture.png
[tl_detector]: ./imgs/tl-detector-ros-graph.png
[waypoint_updater]: ./imgs/waypoint-updater-ros-graph.png
[dbw_node]: ./imgs/dbw-node-ros-graph.png
[graph]: ./imgs/rqtgraph.png
[sim]: ./imgs/simulator.png
[rviz]: ./imgs/rviz.png

## Rubric Points

* The code is successfully built and run on ROS, controlling the simulator.
* The simulated vehicle successfully navigates the full track.

The complete rubric points for this project can be found [here](https://review.udacity.com/#!/rubrics/3058/view).

## Software Installation, Build & Execution

The project has been tested in two different environments: (1) a docker container running Ubuntu Linux, and (2) a native installation of Ubuntu Linux.

### Docker-based installation

The `Dockerfile` provided defines a Ubuntu 16.04 based system running ROS kinetic. First, build a docker image by

```
$ docker build . -t capstone
```

at the top directory of the repository.

Using the above docker image, a container can be run by

```
$ docker run -p 4567:4567 -v "$PWD":/capstone -v "$PWD/../log":/root/.ros/log --rm -it capstone
```

which maps the host's repository directory to the container's `/capstone` directory where the software will be executed. The log directory for the ROS software (`/root/.ros/log`) will be mapped to the host's directory (which must be created beforehand) so that the log files can be accessed from the host during execution and after the ROS software is terminated.

In the container, a few required packages are installed by

```
pip install -r requirements.txt
```

from the directory `/capstone`, although the required packages are likely to be pre-installed by docker build. Note that, the package `Pillow`'s version is changed from `2.2.1` to `6.2.2` for compatibility issues occurring when the docker container is run on MacOS.

In order to build the ROS-based software, the following sequence of commands is used:

```
cd /capstone/ros
catkin_make
source devel/setup.bash
```

in the container. If permission bits of some files are messed, it may be necessary to change them before the workspace can be built by the following command:

```
find . -type f -name "*.sh" -exec chmod u+x {} \;
find . -type f -name "*.py" -exec chmod u+x {} \;
```

Now the software can be run by

```
roslaunch launch/styx.launch
```

after which the simulator (downloaded from [https://github.com/udacity/CarND-Capstone/releases](https://github.com/udacity/CarND-Capstone/releases)) is run and connected to the ROS-based software through port 4567.

### Native installation

Another approach was tested in Ubuntu 18.04 LTS running ROS melodic. If every required package has already been installed (see above), the workspace can be built from the `$REPO_ROOT/ros` directory and then run by the same commands described above.

## System Architecture

The entire system's architecture is illustrated in the following figure. The self-driving software system consists of three distinct pipelines: perception, planning, and control.

![Software Architecture of the System][arch]

Each of the pipelines is implemented in a very simple manner. The perception pipeline is aimed at detecting traffic light detection and classification. The planning pipeline is responsible for loading waypoints (global path plan) and updating them to precisely make a local motion plan. Finally, the control pipeline generates control commands for the vehicle by (1) determining the vehicle's motion following the waypoints using a control algorithm, and then (2) converting the motion command to throttle, steering, and brake command parameters.

## Implementation

### Perception

The traffic light detection node (implemented in `tl_detector.py`) obtains its input from three different topics: `/base_waypoints`, `/iamge_color`,  and `/current_pose`. Based on its algorithm, the node produces waypoints that the vehicle is going to follow, and publishes them to a single topic `/traffic_waypoints`. This structure is shown in the following diagram:

![Input and Output for the Traffic Light Detection Node][tl_detector]

The messages published in the topic `/image_color` contains color images captured by the frontal camera of the simulated (or, in the real HW environment, the self-driving) vehicle. When visulaized by RVIZ, those images look like, for example, the following screenshot.

![Visualization of Camera Input][rviz]

However, instead of incorporating a traffic light detector and classifier based on machine learning, in this implementation the traffic light detector relies on traffic light information given by the simulator through topic `/vehicle/traffic_lights` and a pre-loaded configuration file `sim_traffic_light_config.yaml`. The former gives a list of traffic lights in the map while the latter provides stop line positions associated with the traffic lights.

When the node receives a message containing the global waypoints (via topic `/base_waypoints`), it constructs a `KDTree` for the purpose of determining (in an efficient manner) the waypoint that is closest among them to the current vehicle position (given by `/current_pose`. After this initialization is done, upon receiving a camera image (although the image is not used), the node executes the method `process_traffic_lights()` to determine the waypoint of the upcoming traffic light closest to the vehicle, along with the state of that light. If the light is red or yellow, the waypoint's index (in the base waypoints array) is published at the topic `/traffic_waypoint`. Otherwise, `-1` is published to indicate that no stopping should be planned according to an upcoming traffic light.

### Planning

The global path planning is pre-computed and the corresponding list of waypoints are loaded from the file `$REPO_ROOT/data/wp_yaw_const.csv` (as specifed in `$REPO_ROOT/ros/src/waypoint_loader/launch/waypoint_loader.launch`). It is then published to the topic `/base_waypoints` by the waypoint loader (implemented in `waypoint_loader.py`). The waypoints have the type `Lane`, which encapsulates the vehicle's intended pose (position combined with orientation) and velocity. The position is determined by data read from the file (specifying `x`, `y`, `z`, and `yaw`), while the velocity is set to the maximum velocity given by the ROS parameter (specified in the launch file `waypoint_loader.launch`). Therefore, without any red (or yellow) traffic light in front, the vehicle is (intended to be) driven at the maximum speed allowed.

![Input and Output for the Waypoint Updater Node][waypoint_updater]

The topic `/base_waypoints` is subscribed to by the waypoint updater node, as shown in the above diagram. Besides the base waypoints, the waypoint updater subscribes to `/traffic_waypoint` and `/current_pose`. The former is used to determine whether (and how) to decelerate the vehicle at the upcoming red (or yellow) traffic light, while the latter is used to compute the set of waypoints ahead of the vehicle's current position.

At 50 Hz (set by `rospy.Rate()` in `loop()` method), the node publishes (to topic `/final_waypoints`) the set of waypoints that the vehicle should follow. This consists of a number (designated by `LOOKAHEAD_WPS = 200`) of waypoints ahead of the vehicle, based on `get_closest_waypoint_idx()` returning the index of the base waypoint that is closest to (while ahead of) the vehicle's current position. This index is given to the method `publish_waypoints()`, which adjusts the target velocity of them according to the traffic light information.

When no traffic light (either red or yellow) is detected within the list of base waypoints (within the lookahead range), the base waypoints are used as they are. In contrast, if a stop is necessary, the node plans to decelerate the vehicle by recalculating the linear velocity associated with each of the base waypoints. Finally, those waypoints are publihsed to the topic `/final_waypoints`.

### Control

The node waypoint follower is implemented in C++ (`pure_pursuit.cpp`) in package `waypoint_follower`. This is assumed to implement the pure pursuit algorithm described in [[1]](#references). It receives (from topic `/final_waypoints`) the list of waypoints that the vehicle should follow, and generates a set of twist commands, which comprises the target linear and angular velocity for the vehicle.

![Input and Output of the DBW Node][dbw_node]

The `/twist_command` is subscribed to by the DBW (Driver By Wire) node. This node, illustrated in the above diagram, generates control commands for the throttle, steering, and brake (via topics `/vehicle/throttle_cmd`, `/vehicle/steering_cmd`, and `/vehicle/brake_cmd`, respectively). For this purpose, the node also subscribes to the topics `/current_velocity` and `/vehicle/dbw_enabled`. Note that when `/vehicle/dbw_enabled` is false, this node does not publish any message to the three topics mentioned above. Besides, in order to prevent accumulated errors from causing unstable control behvaiour, the PID controller (as will be described below) is reset every time `/vehicle/dbw_enabled` is disabled (corresponding to situations where a driver takes over the control).

The DBW node publishes the commands at 50 Hz (as set by `rospy.Rate()` in `loop()` method), using a set of controllers defined in `twist_controller.py`. The `Controller` class implemented in the module uses (1) a `YawController` (implemented in `yaw_controller.py`) for steering control, and (2) a PID controller (`PID` implemented in `pid.py`) for throttle and brake control. In addition, a simple low pass filter (implemented in `lowpass.py`) is applied to the monitored current velocity to remove any noisy fluctuation in the measurement.

## Test

To run the simulation integrated with the ROS-based self-driving software, we first run the server (with `socketio`-based server implemented in `styx` package) by `roslaunch` and then execute the simulator. As the simulator communicates with the server, data from the simulator (along with those loaded from files) begin to be published as ROS messages. The following figure shows the resulting task graph produced by `rqt_graph` on the host running the ROS-based software.

![Task Graph Provided by rqtgraph][graph]

When the "camera" option is turned on and "manual" option is turned off, the simulated vehicle starts to follow the waypoints (`/final_waypoints`) published by `/waypoint_updater` by using control algorithms implemented by `/pure_pursuit` and `/dbw_node`. The vehicle drives at the maximum allowed speed (set in the launch description file) unless a red (or yellow) traffic light is visible within the lookahead range. If the vehicle approaches a red traffic light, it slows down (according to the algorithm implemented in `/waypoint_udpater`) and eventually stops at the light. When the light becomes green, the vehicle speeds back up (according to the maximum accleretion specified in the configuration and maximum throttle defined in `twist_controller.py`) to the maximum speed. The following screenshot is taken while the simulated vehicle is stopped at a red traffic light and waiting for the light to turn green.

![Simulator Screenshot][sim]

At first, I tried to run the test on a Macbook Pro running docker. Before integrating the traffic light detector (along with the first and second project walkthroughs), the simulation seemed OK and observations were made where the vehicle faithfully follows the generated trajectory at the speed setting of 40 kph (default value in `waypoint_loader.launch`). However, as I went further to integrate the traffic light detector (with the "camera" option turned on at the simulator), the simulation failed to control the vehicle to follow the intended behaviour. Judging from the fact that even the waypoints are not updated in time, I fancied the failure might have been related to performance issues. (My conjecture is that the performance drop results from the computation load exerted by the simulator, rather than the software system involving docker and ROS.)

I switched to a desktop computer with Ubuntu 18.04 LTS installed, and this time the simulation was successful using docker. Even as I tried a higher speed limit (e.g. 60 kph), no serious problem was observed and the vehicle was controlled as intended. However, if I raise the maximum speed to an even higher limit (e.g. 90 kph), the vehicle begins to veer sideways at relatively steep curves along the highway (although it's not really a steep turn). Experiments showed that stable behaviour could be obtained at a speed as high as 70-80 kph settings.

The deviation from waypoints at higher speeds seemingly results from the control algorithm's characteristics, instead of the computing system's performance bound. One interesting extension might be dynamically adjusting the target speed according to the upcoming trajectory's curvature. Another obvious enhancement would be fine-tuning the pure pursuit implementation or experimenting with another control algorithm such as MPC (model predictive control).

In order to obtain the task graph (using `rqt_graph`) and image screenshot (using `rviz`), I ran the software on a native installation of ROS (on the same computer). Experiments showed that the performance was comparable to the docker case, which largely verifies the conjecture that docker involves very little overhead in containerization.


## References

[1] Coulter, R. C. Implementation of the Pure Pursuit Path Tracking Algorithm. Technical Report, CMU-RI-TR-92-01, Jan. 1992.
