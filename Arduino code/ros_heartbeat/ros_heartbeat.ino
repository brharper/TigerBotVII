/*
 * rosserial Publisher/Subscriber Example
 * Prints "hello world!" periodically, and toggles LED when it sees a message
 *  At initialization, LED is on until connection to ROS is formed, then turns off
 *  When disconnected, LED blinks until connection is formed again
 *  Also, teensy publishes heartbeat topic and subscribes to do_action topic.
 *  Heartbeat topic shows teensy is alive, and do_action comes from heartbeat node,
 *  which will keep track of all teensys.
 *  Required modification: have do_action publish continously, and timestamp received messages,
 *  so that not receiving messages after a timeout can indicate disconnection faster then nh.connected()
 *  (Since nh.connected requires a timeout of ~10 by design)
 * 1/10/17
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
unsigned long publish_time, publish_time_hb;
boolean action_flag=0;  //Wait until heartbeat node confirms go-ahead
int8_t id=2;

ros::NodeHandle nh1;

std_msgs::String str_msg;
std_msgs::Int8 id_msg;  //For Heartbeat status

ros::Publisher chatter1("chatter1", &str_msg);
ros::Publisher heartbeat2("heartbeat2", &id_msg);

char hello1[13] = "hello world!";

//When the subscribed topic sees a message, this function is called
void messageCb1( const std_msgs::Empty& toggle_msg){
  if (action_flag) {
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  }
  else {
    nh1.loginfo("action_flag is false");
  }
}

//For heartbeat, pause or resume actions based on message sent (change action_flag)
void messageCb2( const std_msgs::Bool& bool_msg){
  action_flag=bool_msg.data;
}
ros::Subscriber<std_msgs::Empty> sub1("toggle_led1", &messageCb1 );
ros::Subscriber<std_msgs::Bool> sub2("do_action", &messageCb2 );


void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  nh1.initNode();
  nh1.advertise(chatter1);
  nh1.advertise(heartbeat2);
  nh1.subscribe(sub1);
  nh1.subscribe(sub2);
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
  id_msg.data=id;
  if (millis()-publish_time>500) {
    chatter1.publish( &str_msg );
    publish_time=millis();
  }
  if (millis()-publish_time_hb>100) {
    heartbeat2.publish( &id_msg );
    publish_time_hb=millis();
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
