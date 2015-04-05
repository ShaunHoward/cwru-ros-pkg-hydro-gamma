# irb120_IM_interface

A package that includes code to run the irb120 arm for robot Abby via an interactive marker interface in EECS 376 Mobile Robotics at CWRU.

It is a modified version of the given IM interface example code designed to produce smooth robot-arm motions to converge (when possible) on goals set by an interactive marker.

The joint motion goes from the current joint angles to the solution joint angles smoothly.  When multiple solutions exist, the target solution is chosen rationally.

This is done in consideration of how the arm is mounted on Abby and in consideration of how to approach objects on a table top from above.  (I.e., avoids interference between the arm and the table).

Directions to run this code follow:

1) roslaunch abby_gazebo abby_world.launch

2) rosrun example_interactive_marker IM_6dof_example
  1. in rviz, add an “InteractiveMarker” display, and select topic “example_marker/update.”

3) rosrun irb120_IM_interface irb120_IM_interface joint_states:=abby/joint_states

4) In rviz, move the 6-DOF marker to a desired pose for the tool flange

5) In another window, trigger motion with: rosservice call move_trigger 1


## Example usage
`rosservice call move_trigger 1`
## Running tests/demos
