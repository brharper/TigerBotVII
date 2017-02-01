#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"

/**
 * num is number of Teensys to keep track of
 * timestamp is an array of Time objects, to keep track of when each Teensy sent a heartbeat message
 * timeout is the Duration object (in seconds) of how long without a message this node should go before pausing actions
 * action_flag is the boolean message sent out that all the Teensy nodes subscribe to. If false, Teensys all stop doing what they're doing.
 */

//const int num=8;
//ros::Time timestamp[num];
//ros::Duration timeout(0.5);
float accel_val;
std_msgs::Int32 motor_vel;

/** The callback function for each heartbeat message, which consists of an int.
 * The int identifies which Teensy it came from, and puts the current time into an array at that index
 */
void imuCallback(const sensor_msgs::Imu msg)
{
  accel_val=msg.linear_acceleration.x;
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

  ros::Publisher motor_pub = n.advertise<std_msgs::Int32>("motor_speed", 1000);
  ros::Subscriber sub1 = n.subscribe("imu_data", 1000, imuCallback);

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    motor_vel.data=accel_val*6000;
    motor_pub.publish(motor_vel);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
