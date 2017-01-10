#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

/**
 * num is number of Teensys to keep track of
 * timestamp is an array of Time objects, to keep track of when each Teensy sent a heartbeat message
 * timeout is the Duration object (in seconds) of how long without a message this node should go before pausing actions
 * action_flag is the boolean message sent out that all the Teensy nodes subscribe to. If false, Teensys all stop doing what they're doing.
 */

const int num=2;
ros::Time timestamp[num];
ros::Duration timeout(0.5);
bool action_flag=false;

/** The callback function for each heartbeat message, which consists of an int.
 * The int identifies which Teensy it came from, and puts the current time into an array at that index
 */
void heartbeatCallback(const std_msgs::Int8 msg)
{
  timestamp[msg.data-1] = ros::Time::now();
}

/**
 * If this node has recieved messages from the rosserial nodes recently, it will publish a 'go-ahead' message.
 * If it's been awhile for at least one node, it will set the message to false.
 * The message is published only when the status has changed, so this could probably be replaced with a service.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "heartbeat");

  ros::NodeHandle n;

  ros::Publisher pause_pub = n.advertise<std_msgs::Bool>("do_action", 1000);
  ros::Subscriber sub1 = n.subscribe("heartbeat1", 1000, heartbeatCallback);
  ros::Subscriber sub2 = n.subscribe("heartbeat2", 1000, heartbeatCallback);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {
    bool status_flag=true;
    ros::Time current_time=ros::Time::now();
    for (int i=0; i<num; i++) {
       if ((current_time-timestamp[i])>timeout) {
	  status_flag=false;
       }
    }
    if (status_flag!=action_flag) {
       action_flag=status_flag;
       std_msgs::Bool msg;
       msg.data=action_flag;
       pause_pub.publish(msg);
       ROS_INFO("Status has changed");
    }

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
