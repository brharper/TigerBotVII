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
 *  
 *  Odroid to Teensy: Teensy sends ready, Odroid publishs messages, sends finished, and publishes time, Teensy publishes stats
 *  Teensy to Odroid: Odroid sends ready, Teensy publishes messages, sends finished, and publishes time, Odroid publishes its stats
 *  Both ways at once: After both sent ready, each publishes messages, sends finished, and publishes time. Then both shows stats for receiving.
 *  
 *  Teensy: subscriber for Odroid's ready and finish, publisher for teensy to odroid test and for ready and finish messages. Others are log messages
 *  ODroid: subsciber for Teensy's ready and finish, publisher for odroid to teensy and ready/finish messages.
 *  
 * 1/18/17
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>
unsigned long Odump_begin, Odump_end, Tdump_begin, Tdump_end;
boolean Oready_flag=0, Ofinished_flag=0;
int32_t id=1;
int8_t teensy_id=1;
float Tdump_duration, Odump_duration;
int count=0, message_num=100;

ros::NodeHandle nh1;

std_msgs::Int8 Tready_flag, Tfinished_flag;

std_msgs::Int64 id_msg;
std_msgs::Int16 num_msg;

ros::Publisher pub1('Tdump1', &id_msg);
ros::Publisher pub2("Tready1", &Tready_flag);
ros::Publisher pub3("Tfinished1", &Tfinished_flag);
ros::Publisher pub4("Teensy_info1", &num_msg);


//When the subscribed topic sees a message, this function is called
void messageCb1( const std_msgs::Bool& bool_msg){
  Oready_flag=bool_msg.data;
}

//For heartbeat, pause or resume actions based on message sent (change action_flag)
void messageCb2( const std_msgs::Bool& bool_msg){
  Ofinished_flag=bool_msg.data;
//  if (Ofinished_flag) Odump_end=millis();
}

void messageCb3( const std_msgs::Int64& rec_msg){
  count++;
  //nh1.loginfo("Count happened");
  if (count==1) {Odump_begin=millis();}
  if (count==message_num) Odump_end=millis();
}
ros::Subscriber<std_msgs::Bool> sub1("Oready", &messageCb1 );
ros::Subscriber<std_msgs::Bool> sub2("Ofinished", &messageCb2 );
ros::Subscriber<std_msgs::Int64> sub3("Odump1", &messageCb3 );



void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  int BAUD=57600;
  nh1.getHardware()->setBaud(BAUD);
  nh1.initNode();
  nh1.advertise(pub1);
  nh1.advertise(pub2);
  nh1.advertise(pub3);
  nh1.advertise(pub4);
  nh1.subscribe(sub1);
  nh1.subscribe(sub2);
  nh1.subscribe(sub3);
  digitalWrite(13,HIGH);
  while (!nh1.connected()) {
    nh1.spinOnce();
  }
  digitalWrite(13,LOW);
  nh1.loginfo("I see connection");
  delay(3000);

}

void loop()
{  
  //Odroid to Teensy
  Tready_flag.data=teensy_id;
  pub2.publish(&Tready_flag);
  //Odump_begin=millis();
  while(count!=message_num){
    nh1.spinOnce();
  }
  nh1.spinOnce();
  Ofinished_flag=0;
  
  Odump_duration=(Odump_end-Odump_begin);

  nh1.loginfo("Teensy side finished receiving messages");
  num_msg.data=Odump_duration;    //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data=count;             //Then publish number of messages received
  pub4.publish(&num_msg);
  nh1.spinOnce();
  count=0;
//  Tready_flag.data=false;
//  pub2.publish(&Tready_flag);

  //Teensy to Odroid
  while(!Oready_flag) {
    nh1.spinOnce();
  }
  Oready_flag=0;
  id_msg.data=id;
  
  nh1.loginfo("Starting Teensy message dump");
  Tdump_begin=millis();
  for (int i=0; i<message_num; i++) {
    //id_msg.data=i;    //Make each message unique? Would slow process down slightly
    pub1.publish( &id_msg );
    //nh1.loginfo("Publishing to Tdump");
    nh1.spinOnce();
  }
  nh1.spinOnce();
  Tdump_end=millis();
  Tfinished_flag.data=teensy_id;
  pub3.publish(&Tfinished_flag);
  nh1.spinOnce();
  nh1.loginfo("Message dump finished");
  Tdump_duration=Tdump_end-Tdump_begin;
  nh1.loginfo("Teensy side finished sending messages");
  num_msg.data=Tdump_duration;    //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data=message_num;             //Then publish number of messages sent
  pub4.publish(&num_msg);
//  Tfinished_flag.data=false;
//  pub3.publish(&Tfinished_flag);


  //Both Odroid and Teensy sending data
  Tready_flag.data=teensy_id;
  pub2.publish(&Tready_flag);
  while(!Oready_flag) {
    nh1.spinOnce();
  }
  //Odump_begin=millis();
  nh1.loginfo("Starting simultaneous message dump");
  Tdump_begin=millis();
  for (int i=0; i<message_num; i++) {
    //id_msg.data=i;    //Make each message unique? Would slow process down slightly
    pub1.publish( &id_msg );
    nh1.spinOnce();
  }
  Tdump_end=millis();
  Tfinished_flag.data=teensy_id;
  pub3.publish(&Tfinished_flag);
  nh1.spinOnce();
  nh1.loginfo("Message dump finished");
  Tdump_duration=(Tdump_end-Tdump_begin);
  nh1.loginfo("Teensy side finished sending messages");
  num_msg.data=Tdump_duration;    //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data=message_num;             //Then publish number of messages sent
  pub4.publish(&num_msg);  
  while(count!=message_num){
    nh1.spinOnce();
  }
  nh1.spinOnce();
  //Odump_end=millis();
  Odump_duration=(Odump_end-Odump_begin);
  nh1.loginfo("Teensy side finished receiving messages");
  num_msg.data=Odump_duration;    //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data=count;             //Then publish number of messages received
  pub4.publish(&num_msg);  
//  Tready_flag.data=false;
//  pub2.publish(&Tready_flag);
//  Tfinished_flag.data=false;
//  pub3.publish(&Tfinished_flag);
nh1.loginfo("Finished with test");
  id_msg.data=id;
  nh1.loginfo("Entering loop");
while(1) {
  //pub1.publish(&id_msg);
    nh1.spinOnce();
    //delay(1);
    //nh1.loginfo("In loop now");2

  }



}
