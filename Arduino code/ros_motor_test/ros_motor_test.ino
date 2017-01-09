
/*
   Testing motors using ROS
   Has a publisher since it used past ROS test code- just shows it can do that simultaneously
   Moves motors when it sees a message on the topic it's subscribed to
   Motor movement is movement in one direction for 5 seconds
   (Toggles direction and LED every time it's called)
   Motor code taken from Left_Leg_Motor_Test (TB6), look there for more explanation of code

   10/12/16
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
unsigned long publish_time;

//Left Ankle Up and Down Motion
//Example code (Left_Leg_Motor_Test) uses 12, 15, 20, and 19
//PCB says 24, 26, 25, 27 <- use this
int LAnkleFB_Enable = 24;
int LAnkleFB_A = 26;
int LAnkleFB_B = 25;
int LAnkleFB_HLFB = 27;

ros::NodeHandle nh1;

std_msgs::String str_msg;
ros::Publisher chatter1("chatter1", &str_msg);

char hello1[13] = "hello world!";

//Runs this callback function when sees message from ROS
void messageCb( const std_msgs::Empty& toggle_msg) {
  int value = digitalRead(13);
  digitalWrite(13, HIGH - value); // blink the led

  //Move motor- high flexs up, low is down
  digitalWrite(LAnkleFB_A, HIGH - value); //Set motor direction
  delay(10);
  digitalWrite(LAnkleFB_Enable, HIGH);  //Enable motor
  analogWrite(LAnkleFB_B, 10000);       //Move motor
  delay(5000);
  analogWrite(LAnkleFB_B, 0);           //Stop motor
  digitalWrite(LAnkleFB_Enable, LOW);   //Disable motor

}
ros::Subscriber<std_msgs::Empty> sub1("run_motor", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);

  pinMode(LAnkleFB_HLFB, INPUT_PULLUP);
  pinMode(LAnkleFB_Enable, OUTPUT);
  pinMode(LAnkleFB_A, OUTPUT);
  pinMode(LAnkleFB_B, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(LAnkleFB_B, 14000);

  nh1.initNode();
  nh1.advertise(chatter1);
  nh1.subscribe(sub1);
}

void loop()
{
  str_msg.data = hello1;
  if (millis() - publish_time > 500) {
    chatter1.publish( &str_msg );
    publish_time = millis();
  }

  nh1.spinOnce();
  delay(1);
}
