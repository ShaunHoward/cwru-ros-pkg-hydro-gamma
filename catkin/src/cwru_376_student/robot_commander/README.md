# robot_commander

Team Gamma

This package exists to command the velocity of the robot. 
A velocity scheduler (vel_scheduler) node exists in the package which determines when to 
accelerate, decelerate, or stop the robot. Trapezoidal speed up and trapezoidal
speed down algorithms determine at what rate to speed up or down the robot according to 
the lidar_alarm node and estop_listener node as well as the odometer readings
from the robot. The messages subscribed to are the nav_msgs about odometry, i.e.
"odom", and the "lidar_alarm", "lidar_dist" and "estop_listener" messages
published from package "sensor_utils". The message published is called "cmd_vel"
which are a type of Twist message for robot velocity. It also has a subscriber for a
software halt command. This subscriber subscribes to the boolean "halt_cmd" message.
A message like this can be published within a terminal when the vel_profiler
is run with rosrun in order to stop the robot on demand like estop.

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

Example of how to code the robot to run on a new line segment in vel_scheduler in main():
"velocityPublisher <- publishes "cmd_vel" message
rTimer <- ROS timer
//Initializes a new move along a segment.
initializeNewMove(rTimer);
//Moves the robot along the given segment length which is a float parameter 
//representing meters in length.
moveOnSegment(velocityPublisher, rTimer, segmentLength);"

Example of how to turn the robot on a new segment in vel_scheduler in main():
"velocityPublisher <- publishes "cmd_vel" message
rTimer <- ROS timer
//Rotates the robot to the given endPhi (desired angle of rotation) as a float 
//representing rads via a trapezoidal velocity profile.
rotateToPhi(velocityPublisher, rTimer, endPhi);"

How LIDAR works:

Using the LIDAR, we determine if an object is encountered within a distance of
2.5m to the robot. If so, it will begin to slow down gracefully to a safe distance from 
the object (0.5m). When the object moves out of the way, the robot will resume 
along its original path until the original goal (end of original path). In this way,
it tracks the original path and the distance covered by the slow down path, then
finds the difference between the slow down path and the original where it left off.
Hence, once the robot takes the slow down path, it will resume on the original path if
the object is moved out of its way.

How estop works:

The same type of behavior goes for the estop_listener as for LIDAR. When the estop is turned on,
the robot will stop on its current path. Once the estop is turned off, the robot
will continue on that same original path until the goal destination.

During execution of the vel_scheduler, several messages are spewed out to the
console about the location, movement, etc. of the robot. 

Rotation:

Rotation also uses the trapezoidal speed up and down algorithms but in terms of
angular velocity and odometer rotation. The rotation will stop when the
desired end rotation (endPhi) is met. The rotation is measured in radians. The rotation
speed is determined by constants defined in the header file. These constants
are determined similarly to that of forward velocity but in terms of phi, omega, and
alpha, all with min and max values.
