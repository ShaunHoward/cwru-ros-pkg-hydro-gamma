# sensor_utils

Team Gamma

This package contains files for running the sensors of Jinx.
The sensors utilized are LIDAR and estop.
A lidar_alarm file exists within the package in order to subscribe to messages
from the lidar sent by Jinx.

Please note: In order for the lidar alarm to receive messages from the robot, one may have to change
the lidar_subscriber node handle in order to subscribe to the current robot's lidar sensor.

The lidar_alarm publishes the following messages:
"lidar_alarm": a boolean message as to whether the robot is .5m or less from an object
in front within a -30 to 30 degree range from center
"lidar_dist": a float that describes the closest ping within a -30 degree to 30 degree
range from the front, center of the robot.

An estop_listener file exists within the package in order to subscribe to messages
from the robot about whether the motors are enabled. 

The estop_listener publishes the following messages:
"estop_listener": a boolean message as to whether the robot has motors enabled
This also translates into whether the hardware estop is on.

In order to run the lidar_alarm, execute the following command:

"rosrun sensor_utils lidar_alarm"

In order to run the estop_listener, execute the following command:

"rosrun sensor_utils estop_listener"

One could also call the estop command via terminal like so:

"rostopic pub -r 10 /motors_enabled std_msgs/Bool False"

To revoke this command, one could use the following:

"rostopic pub -r 10 /motors_enabled std_msgs/Bool True"

