/*
    ROS communication test
    ODroid and Teensys take turns dumping messages on each other, and measuring sending/receiving times.
    

    multiple and param- Test for multiple Teensys, parameters taken from ros_launch file for ease of use
    
    v1- Each Teensy dumps messages on their own topic, and ODroid subscribes to all of them.
    
    3 parts to test:
    Odroid to Teensy: Teensy sends ready, Odroid publishs messages and publishes time, Teensy counts all the messages and publishes stats (time and # of msgs)
    Teensy to Odroid: Odroid sends ready, Teensy publishes messages and publishes time, Odroid counts all the messages publishes its stats
    Both ways at once: After both sent ready, each publishes messages, and publishes time. Then both shows stats for receiving.

    Teensy: subscriber for Odroid's ready, publisher for message dump, ready message, and stat message
    ODroid: subsciber for Teensy's ready, publisher for message dump, and ready message

   1/26/17
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
boolean Oready_flag = 0;//, Ofinished_flag = 0;
int param_id;
int32_t id;
int8_t teensy_id;
float Tdump_duration, Odump_duration;
int count = 0, message_num;

ros::NodeHandle nh1;

std_msgs::Int8 Tready_flag;//, Tfinished_flag;

std_msgs::Int64 id_msg;
std_msgs::Int16 num_msg;
std_msgs::Time message_start;

ros::Publisher pub1("Tdump8", &id_msg);
ros::Publisher pub2("Tready8", &Tready_flag);
//ros::Publisher pub3("Tfinished7", &Tfinished_flag);
ros::Publisher pub4("Teensy_info8", &num_msg);
ros::Publisher pub5("Teensy_time8", &message_start);


//When the subscribed topic sees a message, this function is called
void messageCb1( const std_msgs::Bool& bool_msg) {
  Oready_flag = bool_msg.data;
}

//For heartbeat, pause or resume actions based on message sent (change action_flag)
//void messageCb2( const std_msgs::Bool& bool_msg){
//  Ofinished_flag=bool_msg.data;
////  if (Ofinished_flag) Odump_end=millis();
//}

void messageCb3( const std_msgs::Int64& rec_msg) {
  count++;
  //nh1.loginfo("Count happened");
  if (count == 1) {
    Odump_begin = millis();
  }
  if (count == message_num) Odump_end = millis();
}
ros::Subscriber<std_msgs::Bool> sub1("Oready", &messageCb1 );
// ros::Subscriber<std_msgs::Bool> sub2("Ofinished", &messageCb2 );
ros::Subscriber<std_msgs::Int64> sub3("Odump8", &messageCb3 );



void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  int BAUD = 57600;
  nh1.getHardware()->setBaud(BAUD);
  nh1.initNode();
  nh1.advertise(pub1);
  nh1.advertise(pub2);
  //nh1.advertise(pub3);
  nh1.advertise(pub4);
  nh1.advertise(pub5);
  nh1.subscribe(sub1);
  //nh1.subscribe(sub2);
  nh1.subscribe(sub3);
  digitalWrite(13, HIGH);
  while (!nh1.connected()) {
    nh1.spinOnce();
  }
    int param_id, param_mess;
    if (! nh1.getParam("~teensy_id", &param_id, 1)||!nh1.getParam("~message_number", &param_mess,1)) {
    param_id = 1;
    param_mess=100;
  }
teensy_id=param_id;
id=param_id;
message_num=param_mess;

digitalWrite(13, LOW);
nh1.loginfo("I see connection");
delay(3000);

}

void loop()
{
  //Odroid to Teensy
  Tready_flag.data = teensy_id;
  pub2.publish(&Tready_flag);
  //Odump_begin=millis();
  while (count != message_num) {
    nh1.spinOnce();
  }
  nh1.spinOnce();
  // Ofinished_flag=0;

  Odump_duration = (Odump_end - Odump_begin);

  nh1.loginfo("Teensy side finished receiving messages");
  num_msg.data = Odump_duration;  //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data = count;           //Then publish number of messages received
  pub4.publish(&num_msg);
  nh1.spinOnce();
  count = 0;
  //  Tready_flag.data=false;
  //  pub2.publish(&Tready_flag);

  //Teensy to Odroid
  while (!Oready_flag) {
    nh1.spinOnce();
  }
  Oready_flag = 0;
  id_msg.data = id;

  nh1.loginfo("Starting Teensy message dump (and latency test)");
  Tdump_begin = millis();
  for (int i = 0; i < message_num; i++) {
    //id_msg.data=i;    //Make each message unique? Would slow process down slightly
    if (i==(message_num-1)) message_start.data = nh1.now();
    pub1.publish( &id_msg );
    //nh1.loginfo("Publishing to Tdump");
    nh1.spinOnce();
  }
  nh1.spinOnce();
  Tdump_end = millis();
  //Tfinished_flag.data=teensy_id;
  //pub3.publish(&Tfinished_flag);
  nh1.spinOnce();
  nh1.loginfo("Message dump finished");
  Tdump_duration = Tdump_end - Tdump_begin;
  nh1.loginfo("Teensy side finished sending messages");
  num_msg.data = Tdump_duration;  //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data = message_num;           //Then publish number of messages sent
  pub4.publish(&num_msg);
  delay(500);
  pub5.publish(&message_start); //last message's latency

  //  Tfinished_flag.data=false;
  //  pub3.publish(&Tfinished_flag);


  //Both Odroid and Teensy sending data
  Tready_flag.data = teensy_id;
  pub2.publish(&Tready_flag);
  while (!Oready_flag) {
    nh1.spinOnce();
  }
  //Odump_begin=millis();
  nh1.loginfo("Starting simultaneous message dump");
  Tdump_begin = millis();
  for (int i = 0; i < message_num; i++) {
    //id_msg.data=i;    //Make each message unique? Would slow process down slightly
    pub1.publish( &id_msg );
    nh1.spinOnce();
  }
  Tdump_end = millis();
  //Tfinished_flag.data = teensy_id;
  //pub3.publish(&Tfinished_flag);
  nh1.spinOnce();
  nh1.loginfo("Message dump finished");
  Tdump_duration = (Tdump_end - Tdump_begin);
  nh1.loginfo("Teensy side finished sending messages");
  num_msg.data = Tdump_duration;  //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data = message_num;           //Then publish number of messages sent
  pub4.publish(&num_msg);
  while (count != message_num) {
    nh1.spinOnce();
  }
  nh1.spinOnce();
  //Odump_end=millis();
  Odump_duration = (Odump_end - Odump_begin);
  nh1.loginfo("Teensy side finished receiving messages");
  num_msg.data = Odump_duration;  //First publish duration in milliseconds
  pub4.publish(&num_msg);
  num_msg.data = count;           //Then publish number of messages received
  pub4.publish(&num_msg);
  //  Tready_flag.data=false;
  //  pub2.publish(&Tready_flag);
  //  Tfinished_flag.data=false;
  //  pub3.publish(&Tfinished_flag);
  nh1.loginfo("Finished with main test, time for latency test");

  
//  id_msg.data = id;
//   message_start.data = nh1.now();
//   pub1.publish(&id_msg);
//   delay(500);
//   pub5.publish(&message_start);  //Single message latency
//   message_start.data = nh1.now();
//   for (int i=0; i<message_num; i++) {
//    pub1.publish( &id_msg );
//    nh1.spinOnce();
//    if (i==(message_num-1)) message_start.data = nh1.now();
//  }
//  delay(500);
//  pub5.publish(&message_start); //last message's latency


  nh1.loginfo("Entering loop");
  while (1) {
    //pub1.publish(&id_msg);
    nh1.spinOnce();
    //delay(1);
    //nh1.loginfo("In loop now");2

  }



}
