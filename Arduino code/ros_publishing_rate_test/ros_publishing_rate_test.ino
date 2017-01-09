/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
unsigned long publish_time;
int rate_time=1000; //default is once per second

ros::NodeHandle nh1;

std_msgs::String str_msg;
ros::Publisher chatter1("chatter1", &str_msg);

char hello1[13] = "hello!";

void messageCb( const std_msgs::Int16& cmd_msg){
  rate_time=cmd_msg.data;   // change publish rate
}
ros::Subscriber<std_msgs::Int16> sub1("rate_time", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh1.initNode();
  nh1.advertise(chatter1);
  nh1.subscribe(sub1);
}

void loop()
{
  str_msg.data = hello1;
  if (millis()-publish_time>rate_time) {
    chatter1.publish( &str_msg );
    publish_time=millis();
  }
  
  nh1.spinOnce();
  delay(1);
}
