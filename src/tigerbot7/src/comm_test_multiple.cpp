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
int message_num=100;
const int teensy_num=8;
ros::Time Odump_begin, Odump_end, count_begin, count_end, loop_start;
ros::Time Tdump_begin[teensy_num], Tdump_end[teensy_num], Ttest_end[teensy_num];
bool Tready_flag[teensy_num], Tfinished_flag[teensy_num], latency_done[teensy_num];
bool all_ready, count_done, count_started, lat_done;
int64_t id=2;
ros::Duration Tdump_duration[teensy_num], Odump_duration;
ros::Duration count_duration, latency_time;
ros::Duration timeout(5.0);

int count[teensy_num];
float Osecs, count_secs, latency_sec;
float secs[teensy_num];

std_msgs::Bool Oready_flag, Ofinished_flag;
std_msgs::Int64 id_msg;

void Tready_cb( const std_msgs::Int8& rec_msg){
  Tready_flag[rec_msg.data-1]=1;
  ROS_INFO("Received ready from Teensy %d", rec_msg.data);

}

void Tfinished_cb( const std_msgs::Int8& rec_msg){
  Tfinished_flag[rec_msg.data-1]=1;
  //if (Tfinished_flag) Tdump_end=ros::Time::now();
}

void count_cb( const std_msgs::Int64& rec_msg){
  	int num = rec_msg.data;
  count[num-1]=count[num-1]+1;
  Ttest_end[num-1]=ros::Time::now();
  if (count[num-1] ==1) {Tdump_begin[num-1]=ros::Time::now();}
  if (count[num-1]==message_num) {
	Tdump_end[num-1]=ros::Time::now();
	ROS_INFO("Got all messages from Teensy %d", num);
  }
  if (!count_started) {

	count_started=1;
	count_begin=ros::Time::now();
	ROS_INFO("Received message from Teensy %d, count starts now", num);
  }

}
void Ttime_cb1( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[0]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 1 is %f seconds",latency_sec);
  latency_done[0]=true;
}
void Ttime_cb2( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[1]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 2 is %f seconds",latency_sec);
  latency_done[1]=true;
}
void Ttime_cb3( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[2]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 3 is %f seconds",latency_sec);
  latency_done[2]=true;
}
void Ttime_cb4( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[3]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 4 is %f seconds",latency_sec);
  latency_done[3]=true;
}
void Ttime_cb5( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[4]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 5 is %f seconds",latency_sec);
  latency_done[4]=true;
}
void Ttime_cb6( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[5]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 6 is %f seconds",latency_sec);
  latency_done[5]=true;
}
void Ttime_cb7( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[6]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 7 is %f seconds",latency_sec);
  latency_done[6]=true;
}
void Ttime_cb8( const std_msgs::Time& time_msg){
  latency_time=Ttest_end[7]-time_msg.data;
  latency_sec=latency_time.toSec();
  ROS_INFO("Latency: Time to receive pub message from Teensy 8 is %f seconds",latency_sec);
  latency_done[7]=true;
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
  ros::Subscriber sub1a = n.subscribe("Tdump1", 1000, count_cb);
  ros::Subscriber sub1b = n.subscribe("Tready1", 1000, Tready_cb);
  ros::Subscriber sub1c = n.subscribe("Tfinished1", 1000, Tfinished_cb);
  ros::Subscriber sub1d = n.subscribe("Teensy_time1", 1000, Ttime_cb1);

  ros::Subscriber sub2a = n.subscribe("Tdump2", 1000, count_cb);
  ros::Subscriber sub2b = n.subscribe("Tready2", 1000, Tready_cb);
  ros::Subscriber sub2c = n.subscribe("Tfinished2", 1000, Tfinished_cb);
  ros::Subscriber sub2d = n.subscribe("Teensy_time2", 1000, Ttime_cb2);

  ros::Subscriber sub3a = n.subscribe("Tdump3", 1000, count_cb);
  ros::Subscriber sub3b = n.subscribe("Tready3", 1000, Tready_cb);
  ros::Subscriber sub3c = n.subscribe("Tfinished3", 1000, Tfinished_cb);
  ros::Subscriber sub3d = n.subscribe("Teensy_time3", 1000, Ttime_cb3);

  ros::Subscriber sub4a = n.subscribe("Tdump4", 1000, count_cb);
  ros::Subscriber sub4b = n.subscribe("Tready4", 1000, Tready_cb);
  ros::Subscriber sub4c = n.subscribe("Tfinished4", 1000, Tfinished_cb);
  ros::Subscriber sub4d = n.subscribe("Teensy_time4", 1000, Ttime_cb4);

  ros::Subscriber sub5a = n.subscribe("Tdump5", 1000, count_cb);
  ros::Subscriber sub5b = n.subscribe("Tready5", 1000, Tready_cb);
  ros::Subscriber sub5c = n.subscribe("Tfinished5", 1000, Tfinished_cb);
  ros::Subscriber sub5d = n.subscribe("Teensy_time5", 1000, Ttime_cb5);

  ros::Subscriber sub6a = n.subscribe("Tdump6", 1000, count_cb);
  ros::Subscriber sub6b = n.subscribe("Tready6", 1000, Tready_cb);
  ros::Subscriber sub6c = n.subscribe("Tfinished6", 1000, Tfinished_cb);
  ros::Subscriber sub6d = n.subscribe("Teensy_time6", 1000, Ttime_cb6);

  ros::Subscriber sub7a = n.subscribe("Tdump7", 1000, count_cb);
  ros::Subscriber sub7b = n.subscribe("Tready7", 1000, Tready_cb);
  ros::Subscriber sub7c = n.subscribe("Tfinished7", 1000, Tfinished_cb);
  ros::Subscriber sub7d = n.subscribe("Teensy_time7", 1000, Ttime_cb7);

  ros::Subscriber sub8a = n.subscribe("Tdump8", 1000, count_cb);
  ros::Subscriber sub8b = n.subscribe("Tready8", 1000, Tready_cb);
  ros::Subscriber sub8c = n.subscribe("Tfinished8", 1000, Tfinished_cb);
  ros::Subscriber sub8d = n.subscribe("Teensy_time8", 1000, Ttime_cb8);

  ros::Publisher pub1a = n.advertise<std_msgs::Bool>("Oready", 1000 );
  ros::Publisher pub1b = n.advertise<std_msgs::Bool>("Ofinished", 1000 );
  ros::Publisher pub1 = n.advertise<std_msgs::Int64>("Odump1", 1000);
  ros::Publisher pub2 = n.advertise<std_msgs::Int64>("Odump2", 1000);
  ros::Publisher pub3 = n.advertise<std_msgs::Int64>("Odump3", 1000);
  ros::Publisher pub4 = n.advertise<std_msgs::Int64>("Odump4", 1000);
  ros::Publisher pub5 = n.advertise<std_msgs::Int64>("Odump5", 1000);
  ros::Publisher pub6 = n.advertise<std_msgs::Int64>("Odump6", 1000);
  ros::Publisher pub7 = n.advertise<std_msgs::Int64>("Odump7", 1000);
  ros::Publisher pub8 = n.advertise<std_msgs::Int64>("Odump8", 1000);



  ros::Rate loop_rate(10);
  all_ready=0; count_done=0;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {
	  
	  //Odroid to Teensy
	  ROS_INFO("Waiting on Teensy");
	  while(!all_ready){
	    all_ready=1;
	    for (int i=0; i<teensy_num; i++) {
		if (!Tready_flag[i]) all_ready=0;
	    }
	    ros::spinOnce();
	  }
	  all_ready=0;
	  for (int i=0; i<teensy_num; i++) {
		Tready_flag[i]=0;
	  }
	  id_msg.data=0;
	  
	  ROS_INFO("Starting ODroid message dump");
	  Odump_begin=ros::Time::now();
	  for (int i=0; i<message_num; i++) {
		//id_msg.data=i;    //Make each message unique? Would slow process down slightly
		pub1.publish(id_msg );
		pub2.publish(id_msg );
		pub3.publish(id_msg );
		pub4.publish(id_msg );
		pub5.publish(id_msg );
		pub6.publish(id_msg );
		pub7.publish(id_msg );
		pub8.publish(id_msg );
		ros::spinOnce();
	  }
	  Odump_end=ros::Time::now();
	  Ofinished_flag.data=true;
	  pub1b.publish(Ofinished_flag);
	  ros::spinOnce();
	  ROS_INFO("ODroid dump finished");
	  Odump_duration=Odump_end-Odump_begin;
	  Osecs=Odump_duration.toSec();
	  ROS_INFO("Odroid dump: Odroid took %f seconds to send %d messages to all Teensys", Osecs, message_num);
	  
	  //Teensy to Odroid
	  Oready_flag.data=true;
	  pub1a.publish(Oready_flag);
	  //Odump_begin=millis();
	  loop_start=ros::Time::now();
	  while(!count_done){
	    count_done=1;
	    for (int i=0; i<teensy_num; i++) {
		if (count[i]!=message_num) count_done=0;
	    }
	    ros::spinOnce();
		//If takes too long, end it now- some messages probably dropped
	    if ((ros::Time::now() - loop_start)> timeout) count_done=1;
	  }
	  count_started=0;
	  count_end=ros::Time::now();
	  count_duration=count_end-count_begin;
	  count_secs=count_duration.toSec();
	  ros::spinOnce();
	  for (int i=0; i<teensy_num; i++) {
		Tfinished_flag[i]=0;
		Tdump_duration[i]=Tdump_end[i]-Tdump_begin[i];
		secs[i]=Tdump_duration[i].toSec();
	  }
	  //Tdump_duration=(Tdump_end-Tdump_begin);
	  //secs=Tdump_duration.toSec();
	  ROS_INFO("Teensy dump: Odroid took %f seconds total to receive %d messages from each Teensy", count_secs, count[1]);
	  for (int i=0; i<teensy_num; i++) {
		ROS_INFO("Teensy dump: Odroid took %f seconds to receive %d messages from Teensy%d", secs[i], count[i],(i+1));
	  }
	  while(!lat_done){
	    lat_done=1;
	    for (int i=0; i<teensy_num; i++) {
		if (!latency_done[i]) lat_done=0;
	    }
	    ros::spinOnce();
	  }
	  
	  //Both Odroid and Teensy sending data
	  Oready_flag.data=true;
	  count_done=0;
	  for (int i=0; i<teensy_num; i++) {
		count[i]=0;
	  }
	  pub1a.publish(Oready_flag);
	  while(!all_ready){
	    all_ready=1;
	    for (int i=0; i<teensy_num; i++) {
		if (!Tready_flag[i]) all_ready=0;
	    }
	    ros::spinOnce();
	  }
	  all_ready=0;
	  for (int i=0; i<teensy_num; i++) {
		Tready_flag[i]=0;
	  }
	  ROS_INFO("Starting message dump");
	  Odump_begin=ros::Time::now();
	  for (int i=0; i<message_num; i++) {
		//id_msg.data=i;    //Make each message unique? Would slow process down slightly
		pub1.publish(id_msg );
		pub2.publish(id_msg );
		pub3.publish(id_msg );
		pub4.publish(id_msg );
		pub5.publish(id_msg );
		pub6.publish(id_msg );
		pub7.publish(id_msg );
		pub8.publish(id_msg );
		ros::spinOnce();
	  }
	  Odump_end=ros::Time::now();
	  Ofinished_flag.data=true;
	  pub1b.publish(Ofinished_flag);
	  ros::spinOnce();
	  ROS_INFO("Message dump finished");
	  Odump_duration=(Odump_end-Odump_begin);
	  Osecs=Odump_duration.toSec();
	  ROS_INFO("ODroid and Teensy dump: Odroid took %f seconds to send %d messages to all Teensys", Osecs, message_num);
	  loop_start=ros::Time::now();
	  while(!count_done){
	    count_done=1;
	    for (int i=0; i<teensy_num; i++) {
		if (count[i]<message_num) count_done=0;
	    }
	    ros::spinOnce();
	    if ((ros::Time::now() - loop_start)> timeout) count_done=1;
	  }
	  ros::spinOnce();
	  count_started=0;
	  count_done=0;
	  count_end=ros::Time::now();
	  count_duration=count_end-count_begin;
	  count_secs=count_duration.toSec();
	  all_ready=0;
	  for (int i=0; i<teensy_num; i++) {
		Tready_flag[i]=0;
		Tfinished_flag[i]=0;
		Tdump_duration[i]=Tdump_end[i]-Tdump_begin[i];
		secs[i]=Tdump_duration[i].toSec();
	  }

	  ROS_INFO("Odroid and Teensy dump: Odroid took %f seconds total to receive %d messages from each Teensy", count_secs, count[1]);
	  for (int i=0; i<teensy_num; i++) {
		ROS_INFO("Odroid and Teensy dump: Odroid took %f seconds to receive %d messages from Teensy%d", secs[i], count[i],(i+1));
	  }
	 
	  ROS_INFO("Relaunch nodes to repeat test");

	while(ros::ok()) {
	  pub1.publish( id_msg );
	  ros::spinOnce();
	}

  }


  return 0;
}
