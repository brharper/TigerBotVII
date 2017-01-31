#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include "std_msgs/Int64.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>

#include <sstream>
ros::Time Odump_begin, Odump_end, Tdump_begin, Tdump_end, Ttest_end;
bool Tready_flag=0, Tfinished_flag=0;
int64_t id=2;
ros::Duration Tdump_duration, Odump_duration, latency_time;
int count=0;
int message_num=100;
float secs, latency_sec;

std_msgs::Bool Oready_flag, Ofinished_flag;
std_msgs::Int64 id_msg;

void Tready_cb( const std_msgs::Bool& bool_msg){
  Tready_flag=bool_msg.data;

}

void Tfinished_cb( const std_msgs::Bool& bool_msg){
  Tfinished_flag=bool_msg.data;
  //if (Tfinished_flag) Tdump_end=ros::Time::now();
}

void count_cb( const std_msgs::Int64& rec_msg){
  count++;
  Ttest_end=ros::Time::now();
  if (count ==1) {Tdump_begin=ros::Time::now();}
  if (count==message_num) Tdump_end=ros::Time::now();

}

void Ttime_cb( const std_msgs::Time& time_msg){
  latency_time=Ttest_end-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Time to receive pub message is %f seconds",latency_sec);
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "comm_test");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Subscriber sub1 = n.subscribe("Tdump", 1000, count_cb);
  ros::Subscriber sub2 = n.subscribe("Tready", 1000, Tready_cb);
  ros::Subscriber sub3 = n.subscribe("Tfinished", 1000, Tfinished_cb);
  ros::Subscriber sub4 = n.subscribe("Teensy_time1", 1000, Ttime_cb);
  
  ros::Publisher pub1 = n.advertise<std_msgs::Bool>("Oready", 1000 );
  ros::Publisher pub2 = n.advertise<std_msgs::Bool>("Ofinished", 1000 );
  ros::Publisher pub3 = n.advertise<std_msgs::Int64>("Odump", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {
	  
	  //Odroid to Teensy
	  ROS_INFO("Waiting on Teensy");
	  while(!Tready_flag){
		  ros::spinOnce();
	  }
	  Tready_flag=0;
	  id_msg.data=0;
	  
	  ROS_INFO("Starting ODroid message dump");
	  Odump_begin=ros::Time::now();
	  for (int i=0; i<message_num; i++) {
		//id_msg.data=i;    //Make each message unique? Would slow process down slightly
		pub3.publish(id_msg );
		ros::spinOnce();
	  }
	  Odump_end=ros::Time::now();
	  Ofinished_flag.data=true;
	  pub2.publish(Ofinished_flag);
	  ros::spinOnce();
	  ROS_INFO("ODroid dump finished");
	  Odump_duration=Odump_end-Odump_begin;
	  secs=Odump_duration.toSec();
	  ROS_INFO("Odroid dump: Odroid took %f seconds to send %d messages", secs, message_num);
	  
	  //Teensy to Odroid
	  Oready_flag.data=true;
	  pub1.publish(Oready_flag);
	  //Odump_begin=millis();
	  while(count!=message_num){
		ros::spinOnce();
	  }
	  ros::spinOnce();
	  Tfinished_flag=0;
	  Tdump_duration=(Tdump_end-Tdump_begin);
	  secs=Tdump_duration.toSec();
	  ROS_INFO("Teensy dump:Odroid took %f seconds to receive %d messages", secs, count);
	  
	  
	  //Both Odroid and Teensy sending data
	  Oready_flag.data=true;
	  count=0;
	  pub1.publish(Oready_flag);
	  while(!Tready_flag) {
		ros::spinOnce();
	  }
	  ROS_INFO("Starting message dump");
	  Odump_begin=ros::Time::now();
	  for (int i=0; i<message_num; i++) {
		//id_msg.data=i;    //Make each message unique? Would slow process down slightly
		pub3.publish( id_msg );
		ros::spinOnce();
	  }
	  Odump_end=ros::Time::now();
	  Ofinished_flag.data=true;
	  pub2.publish(Ofinished_flag);
	  ros::spinOnce();
	  ROS_INFO("Message dump finished");
	  Odump_duration=(Odump_end-Odump_begin);
	  secs=Odump_duration.toSec();
	  ROS_INFO("ODroid and Teensy dump: Odroid took %f seconds to send %d messages", secs, message_num);
	  while(count!=message_num){
		ros::spinOnce();
	  }
	  ros::spinOnce();
	  Tfinished_flag=0;
	  Tdump_duration=(Tdump_end-Tdump_begin);
	  secs=Tdump_duration.toSec();
	  ROS_INFO("ODroid and Teensy dump: Odroid took %f seconds to receive %d messages", secs, count);
	 
	  ROS_INFO("Relaunch nodes to repeat test");

	while(ros::ok()) {
	  pub3.publish( id_msg );
	  ros::spinOnce();
	}

  }


  return 0;
}
