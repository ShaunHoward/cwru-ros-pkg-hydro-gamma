# des_state_generator

Team Gamma

The des state generator package consists of a desired state generator and a path sender. Both of these files, one of which is a class, are ROS nodes. The desired state generator publishes a "desState" message and the path sender sends files to a path segment queue that the desired state generator can interpret. The state generator takes this path segment queue and expands the given values of x,y,phi into line and spin-in-place path segments. In this way, the state generator currently produces states for both turning in place and moving forward. 

The desired state generator determines where the next desired location (pose) of the robot should be based on the latest odometer update from the robot. The difference in current odom and the initial odom at the beginning of the current path segment determines how much distance or spin is left to be met. Once the necessary criteria is met (tolerance) for the current pose of the robot, a halt state is produced by the robot until the next path segment is unpacked. This data is all attained through the path sender file, so the state generator requires a path sender to acquire the desired path.

This state generator also accounts for if an object is in front of the robot via lidar. If such is true, the robot will stop moving. Once the object has moved out of the way, the robot will resume operation. The same goes for estop and software halting, via the sensor utils package.

The path sender can be run with the following command:

rosrun des_state_generator path_sender

The desired state generator can be run with the following command:

rosrun des_state_generator des_state_generator

Many helpful info messages will print to the console as the state generator extracts and executes path segments. When there are no path segments in the queue, the generator will signal so via terminal.
