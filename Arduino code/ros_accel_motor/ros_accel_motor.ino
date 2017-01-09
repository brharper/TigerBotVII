/*
   Testing motors using ROS
   Has a publisher since it used past ROS test code
   Moves motors when it sees a message on the topic it's subscribed to
   Motor movement is movement in one direction for 5 seconds
   (Toggles direction and LED every time it's called)
   Motor code taken from Left_Leg_Motor_Test, look there for more explanation of code
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
unsigned long publish_time;

//Left Ankle Up and Down Motion
//Example code (Left_Leg_Motor_Test) says 12, 15, 20, and 19
//PCB says 24, 26, 25, 27 <- use this
int LAnkleFB_Enable = 24;
int LAnkleFB_A = 26;
int LAnkleFB_B = 25;
int LAnkleFB_HLFB = 27;
int motor_vel;

ros::NodeHandle nh1;

std_msgs::Int16 speed_printout;
ros::Publisher chatter1("chatter1", &speed_printout);

char hello1[13] = "hello world!";

//Runs this callback function when sees message from ROS
void messageCb( const std_msgs::Int16& motor_speed) {
  int value = digitalRead(13);
  digitalWrite(13, HIGH - value); // blink the led

  //Move motor- high flexs up, low is down
  //Direction depends on sign of motor_speed
  //analogWrite only affects duty cycle, not frequency- set at 50%
  analogWrite(LAnkleFB_B, 127);
  digitalWrite(LAnkleFB_A, HIGH - (motor_speed.data<0)); //Set motor direction
  delay(10);
  analogWriteFrequency(LAnkleFB_B, motor_speed.data);       //Move motor
  delay(0);


}
ros::Subscriber<std_msgs::Int16> sub1("motor_speed", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);

  pinMode(LAnkleFB_HLFB, INPUT_PULLUP);
  pinMode(LAnkleFB_Enable, OUTPUT);
  pinMode(LAnkleFB_A, OUTPUT);
  pinMode(LAnkleFB_B, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(LAnkleFB_B, 50000);
  //Have motor enabled, but off
  analogWrite(LAnkleFB_B, 0);  
  digitalWrite(LAnkleFB_Enable, HIGH);
  Serial.begin(9800);

  nh1.initNode();
  nh1.advertise(chatter1);
  nh1.subscribe(sub1);
}

void loop()
{
//  speed_printout.data = motor_speed;
//  if (millis() - publish_time > 500) {
//    chatter1.publish( &speed_printout );
//    publish_time = millis();
//  }
//
//  nh1.spinOnce();
//  delay(1);
for (int i=1; i<=5; i++){
  motor_vel=i*50;
  analogWrite(LAnkleFB_B, motor_vel);
  delay(1000);
  Serial.print("analogWrite:");
  Serial.println(motor_vel);
}

analogWrite(LAnkleFB_B, 127);

for (int j=1; j<=5; j++){
  motor_vel=j*10000;
  analogWriteFrequency(LAnkleFB_B, motor_vel);
  delay(1000);
  Serial.print("analogWriteFrequency:");
  Serial.println(motor_vel);
}
  digitalWrite(LAnkleFB_Enable, LOW);
  while(1){}

}
