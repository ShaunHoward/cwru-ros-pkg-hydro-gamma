# robot_commander

Team Gamma

This package exists to command the velocity of the robot. 
A velocity scheduler (vel_scheduler) node exists in the package which determines when to 
accelerate, decelerate, or stop the robot. A second steering velocity profiler (SteerVelProfiler) library class exists in the package in order to command the robot velocity via desired states (i.e. des_state_generator package). This class is not a node and therefore does not have a main method. It simply computes the desired next values for forward and turning velocities. Most of the documentation below pertains the velocity scheduler, however, some of the description does pertain to the steering velocity profiler.

Trapezoidal speed up and trapezoidal speed down algorithms determine at what rate
to speed up or down the robot according to the lidar_alarm node and estop_listener node as well as the odometer readings
from the robot. The messages subscribed to are the nav_msgs about odometry, i.e.
"odom", and the "lidar_alarm", "lidar_dist" and "estop_listener" messages
published from package "sensor_utils". These messages do not affect the steering velocity profiler. The message published is called "cmd_vel"
which is a type of Twist message for robot velocity. It also has a subscriber for a
software halt command. This subscriber subscribes to the boolean "halt_cmd" message.
A message like this can be published within a terminal when the vel_profiler
is run with rosrun in order to stop the robot on-demand like a software estop.

Please note: One may have to change the velocityPublisher advertisement as well as the odometry subscriber
handle in order for the robot to respond to the vel_profiler.

To run vel_profiler:

"rosrun robot_commander vel_profiler"

To publish software halt command:

"rostopic pub /halt_cmd std_msgs/Bool True"

To revoke halt command:

"rostopic pub /halt_cmd std_msgs/Bool False"

Either line segments or rotation segments(radians) should be specified in the main method
to run the velocity scheduler. Each time a move is initialized, the coordinates
are set to the most recent coordinates given by the odometer callback to track
current position. The rotation is set according to the odometer as well.

During execution of the vel_scheduler, several messages are spewed out to the
console about the location, movement, etc. of the robot. These help to understand the current actions of the robot as well as how close the robot is to its goal destination.

Example of how to code the robot to run on a new line segment in vel_scheduler in main():
"velocityPublisher <- publishes "cmd_vel" message
rTimer <- ROS timer
//Moves the robot along the given segment length which is a float parameter 
//representing meters in length.
moveOnSegment(velocityPublisher, rTimer, segmentLength);"

Example of how to turn the robot to a new rotation in vel_scheduler in main():
"velocityPublisher <- publishes "cmd_vel" message
rTimer <- ROS timer
//Rotates the robot to the given endPhi (desired angle of rotation) as a float 
//representing rads via a trapezoidal velocity profile.
rotateToPhi(velocityPublisher, rTimer, endPhi);"

Both moves and rotations can be used in combination in order to navigate around turns and reach certain destinations via dead-reckoning. A simple example that will move on a segment and then rotate the robot is as follows:
"velocityPublisher <- publishes "cmd_vel" message
rTimer <- ROS timer
//Moves the robot along the given segment length which is a float parameter 
//representing meters in length.
moveOnSegment(velocityPublisher, rTimer, segmentLength);
//Rotates the robot to the given endPhi (desired angle of rotation) as a float 
//representing rads via a trapezoidal velocity profile.
rotateToPhi(velocityPublisher, rTimer, endPhi);"

How LIDAR works:

Using the LIDAR, we determine if an obstacle is encountered within a distance of
2.5m to the robot. If so, it will begin to slow down gracefully to a safe distance from 
the obstacle (0.5m). When the obstacle moves out of the way, the robot will resume 
along its original path until the original goal (end of original path) is reached. In this way,
it tracks the original path and the distance covered by the slow-down path, then
finds the difference between the slow-down path and the original where it left off.
Hence, after the robot completes the slow down path, it will resume on the original path if
the obstacle is moved out of its way.

Messages will be printed to the console about the closest ping distance from the lidar sensor
as the lidar_alarm node runs.

How estop works:

The same type of behavior goes for the estop_listener as for LIDAR. When the estop is turned on,
the robot will stop on its current path. Once the estop is turned off, the robot
will continue on that same original path until the goal destination is reached.

Messages will be printed to the console about whether estop is on or off as the estop_listener runs.

Rotation:

Rotation also uses the trapezoidal speed up and down algorithms but in terms of
angular velocity and odometer rotation. The rotation will stop when the
desired end rotation (endPhi) is met. The rotation is measured in radians. The rotation
speed is determined by constants defined in the header file. These constants
are determined similarly to that of forward velocity but in terms of phi, omega, and
alpha, all with min and max values.

Note: The robot will rotate in circles without losing its place in rotation. This means that
the robot will continue turning until its endPhi is met, and will only slightly overshoot that
goal due to inaccuracies in the odometer or slowness of refresh rate. For the cases where rotation
changes from positive to negative or vice versa, the robot will not get confused, but instead will
handle the transition as if it was naturally continuing in the same direction as before.
