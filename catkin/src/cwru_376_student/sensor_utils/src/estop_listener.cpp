#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <iostream>
#include <fstream>
#include <cwru_base/cRIOSensors.h>
#include <std_msgs/Bool.h>

bool motorsEnabled;
ros::Publisher estopPublisher;

/**
 * In order to call the estop command from terminal, use the following line:
 * rostopic pub -r 10 /motors_enabled std_msgs/Bool False
 */
void estopCallback(const std_msgs::Bool::ConstPtr& motorsEnabled) 
{
    if (motorsEnabled->data == true)
      ROS_INFO("Estop is off."); // means motors are ENABLED
    else if (motorsEnabled->data == false)
      ROS_INFO("Estop is on."); // means motors are ENABLED
    
   std_msgs::Bool estopMessage;
   estopMessage.data = !(motorsEnabled->data);
   estopPublisher.publish(estopMessage);
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "estop_listener");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nodeHandle;
        
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::Bool>("estop_listener", 1);
    estopPublisher = publisher; // let's make this global, so callback can use it

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber subscriber = nodeHandle.subscribe("motors_enabled", 1, estopCallback);

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}

