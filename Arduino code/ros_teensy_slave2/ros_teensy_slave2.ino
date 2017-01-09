/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
unsigned long publish_time;

ros::NodeHandle nh2;

std_msgs::String str_msg;
ros::Publisher chatter2("chatter2", &str_msg);

char hello2[12] = "hello user!";

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
ros::Subscriber<std_msgs::Empty> sub2("toggle_led2", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh2.initNode();
  nh2.advertise(chatter2);
  nh2.subscribe(sub2);
}

void loop()
{
  str_msg.data = hello2;
  if (millis()-publish_time>1000) {
    chatter2.publish( &str_msg );
    publish_time=millis();
  }
  
  nh2.spinOnce();
  delay(1);
}
