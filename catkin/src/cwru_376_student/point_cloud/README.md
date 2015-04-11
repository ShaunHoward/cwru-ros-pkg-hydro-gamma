# point_cloud

Team Gamma

This package contains a program to find a can on a table using a Kinect point cloud. The program will load in the test pcd file that was saved when Professor Newman ran Gazebo with cwruBot and the can on the table. This snapshot is loaded into memory and then processed with several different steps. Each step of processing should be run in tandem with Rviz to fit a can model to the point cloud points representing the can on the table. A gradient descent optimization algorithm is used to register the can model to fit with the point cloud representation of the can. The program publishes a "process_mode" service that allows the user to run each process mode/step and get closer to the desired goal.

First, one should start up roscore and rviz. Then run the "find_can" program in this package.
Set the fixed frame to "world" in rviz and reveal the kinect_pointcloud and plane_model topics. 
Change the color of the kinect_pointcloud to represent the z-axis.
Select a patch of points from the table with the "publish selected points" tool in rviz.

run: rosservice call process_mode 0

This will find the desired plane of the table using a published selection of points via Rviz.

Then select a patch of points from the can this time and run process_mode 1.

This will find the ponts above the plane. Next run process_mode 2. 

This will make a 3-D can model and place it at an estimated origin in the rviz scene.\

Finally, run process_mode 3 approximately 7 times to get the can model to fit very closely with the point
cloud representation.

This is the desired outcome of the program and it works well.
