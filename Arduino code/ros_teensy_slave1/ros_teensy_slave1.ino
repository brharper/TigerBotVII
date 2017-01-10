/*
 * rosserial Publisher/Subscriber Example
 * Prints "hello world!" periodically, and toggles LED when it sees a message
 * Second teensy has similar code, just with a different message, and different
 *  topics it publishes/subscribes to.
 *  At initialization, LED is on until connection to ROS is formed, then turns off
 *  When disconnected, LED blinks until connection is formed again
 * 1/7/17
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
unsigned long publish_time;

ros::NodeHandle nh1;

std_msgs::String str_msg;
ros::Publisher chatter1("chatter1", &str_msg);

char hello1[13] = "hello world!";

//When the subscribed topic sees a message, this function is called
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
ros::Subscriber<std_msgs::Empty> sub1("toggle_led1", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  nh1.initNode();
  nh1.advertise(chatter1);
  nh1.subscribe(sub1);
  digitalWrite(13,HIGH);
  while (!nh1.connected()) {
    nh1.spinOnce();
  }
  digitalWrite(13,LOW);
  nh1.loginfo("I see connection");

}

void loop()
{
  //The publisher publishes a message at 2Hz
  str_msg.data = hello1;
  if (millis()-publish_time>500) {
    chatter1.publish( &str_msg );
    publish_time=millis();
  }
  
  nh1.spinOnce();
  delay(1);
  if(!nh1.connected()) {
    for (int i=0; i<3; i++) {
      digitalWrite(13,HIGH);
      delay(100);
      digitalWrite(13,LOW);
      delay(100);
    }
    delay(300);
  }
}
