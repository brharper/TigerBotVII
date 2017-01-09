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
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

unsigned long publish_time;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//Left Ankle Up and Down Motion
//Example code (Left_Leg_Motor_Test) says 12, 15, 20, and 19
//PCB says 24, 26, 25, 27 <- use this
int LAnkleFB_Enable = 24;
int LAnkleFB_A = 26;
int LAnkleFB_B = 25;
int LAnkleFB_HLFB = 27;

ros::NodeHandle nh1;

std_msgs::Int16 speed_printout;
ros::Publisher chatter1("chatter1", &speed_printout);

char hello1[13] = "hello world!";

Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);


//Runs this callback function when sees message from ROS
void messageCb( const std_msgs::Int16& motor_speed) {
  int value = digitalRead(13);
  digitalWrite(13, HIGH - value); // blink the led

  //Move motor- high flexs up, low is down
  //Direction depends on sign of motor_speed
  //analogWrite only affects duty cycle, not frequency- set at 50%
  analogWrite(LAnkleFB_B, 127)
  digitalWrite(LAnkleFB_A, HIGH - (motor_speed<0)); //Set motor direction
  delay(10);
  analogWriteFrequency(LAnkleFB_B, motor_speed);       //Move motor
  delay(0);


}
ros::Subscriber<std_msgs::Empty> sub1("motor_speed", &messageCb );

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

  nh1.initNode();
  nh1.advertise(chatter1);
  nh1.subscribe(sub1);

  accel.begin();
  mag.begin();
}

void loop()
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }

  Serial.println(F(""));
  delay(1000);
  speed_printout.data = motor_speed;
  if (millis() - publish_time > 500) {
    chatter1.publish( &speed_printout );
    publish_time = millis();
  }

  nh1.spinOnce();
  delay(1);
}
