/*
 * rosserial Publisher/Subscriber Example
 * Prints "hello world!" periodically, and toggles LED when it sees a message
 *  At initialization, LED is on until connection to ROS is formed, then turns off
 *  When disconnected, LED blinks until connection is formed again
 *  Also, teensy publishes heartbeat topic and subscribes to do_action topic.
 *  heartbeat topic shows teensy is alive, and do_action comes from heartbeat node,
 *  which will keep track of all teensys.
 *  Modification of ros_heartbeat.ino, just to test loading several topics on a Teensy
 *  and publish at different delays
 *  Timer interrupts were set up to test if interrupts affects anything
 * 1/28/17
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
unsigned long publish_time, publish_time_hb;
boolean action_flag=0;  //Wait until heartbeat node confirms go-ahead
int8_t id;
int loop_delay;
boolean more_topics;

ros::NodeHandle nh1;

std_msgs::String str_msg;
std_msgs::Int8 id_msg;  //For heartbeat status
std_msgs::Float64 num_msg;
std_msgs::Int64 mum_msg;

ros::Publisher chatter1("chatter1", &num_msg);
ros::Publisher chatter2("chatter2", &mum_msg);
ros::Publisher chatter3("chatter3", &mum_msg);
ros::Publisher chatter4("chatter4", &mum_msg);
ros::Publisher chatter5("chatter5", &mum_msg);
ros::Publisher heartbeat("heartbeat8", &id_msg);
//ros::Publisher heartbeat2("heartbeat2", &id_msg);
//ros::Publisher heartbeat3("heartbeat3", &id_msg);


char hello1[13] = "hello world!";

IntervalTimer myTimer, myTimer2;

//When the subscribed topic sees a message, this function is called
void messageCb1( const std_msgs::Empty& toggle_msg){
  if (action_flag) {
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  }
//  else {
//    nh1.loginfo("action_flag is false");
//  }
//heartbeat.publish( &id_msg );
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
  nh1.advertise(chatter2);
  nh1.advertise(chatter3);
  nh1.advertise(chatter4);
  nh1.advertise(chatter5);
  nh1.advertise(heartbeat);
//  nh1.advertise(heartbeat2);
//  nh1.advertise(heartbeat3);
  nh1.subscribe(sub1);
  nh1.subscribe(sub2);
  digitalWrite(13,HIGH);
  while (!nh1.connected()) {
    nh1.spinOnce();
  }
  int param_id, loop_id, more_id;
  if (! nh1.getParam("~teensy_id", &param_id, 1)) {
    param_id = 1;
  }
  if (! nh1.getParam("~loop_delay", &loop_id, 1)) {
    loop_id = 20000;
  }
  if (! nh1.getParam("~more_topics", &more_id, 1)) {
    more_id = 1;
  }
  id=param_id;
  loop_delay=loop_id;
  more_topics=more_id;
  digitalWrite(13,LOW);
  myTimer.begin(spinner, 100000);   //Calls every 100 ms
  myTimer2.begin(publishing, loop_delay);   //Calls every 20 ms
  nh1.loginfo("I see connection");

}

void spinner(){
  nh1.spinOnce();
  //nh1.loginfo("spinner called");
}

void publishing(){
  if (more_topics) {
     chatter1.publish( &num_msg );
    chatter2.publish( &mum_msg );
    chatter3.publish( &mum_msg );
    chatter4.publish( &mum_msg );
    chatter5.publish( &mum_msg );
  }
    heartbeat.publish( &id_msg );
//    heartbeat2.publish( &id_msg );
//    heartbeat3.publish( &id_msg );
}

void loop()
{
  //The publisher publishes a message at 2Hz
  str_msg.data = hello1;
  num_msg.data=1;
  id_msg.data=id;
//  if (millis()-publish_time>500) {
//    chatter1.publish( &num_msg );
//    chatter2.publish( &mum_msg );
//    chatter3.publish( &mum_msg );
//    chatter4.publish( &mum_msg );
//    chatter5.publish( &mum_msg );

//    publish_time=millis();
//  }
 // if (millis()-publish_time_hb>5) {
//    heartbeat.publish( &id_msg );
//    heartbeat2.publish( &id_msg );
//    heartbeat3.publish( &id_msg );
//    publish_time_hb=millis();
//  }
  
  //nh1.spinOnce();
  //delay(15);
  if(!nh1.connected()) {
    for (int i=0; i<3; i++) {
      digitalWrite(13,HIGH);
      delay(200);
      digitalWrite(13,LOW);
      delay(200);
    }
    delay(500);
  }
}
