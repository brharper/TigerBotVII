/* Tigerbot Daisy Chain Motor Test Sequence - Left Leg
 * Leslie Bowen 3/27/2016
 * 
 * 9/17/2016: Testing motors of left leg detached
 */

 // Motor Definitions/Pin Designations
 // This sets each of the motor's 4 signals to a pin on the Teensy, these pin assingments are flexible
 // except that each b signal should correspond to a PWM capable pin. A list of PWM capable pins is given
 // on the Teensy spec sheet and the PJRC website https://www.pjrc.com/teensy/pinout.html

 //Left Knee Signals
 int LKneeFB_Enable = 0;
 int LKneeFB_A = 1;
 int LKneeFB_HLFB = 2;
 int LKneeFB_B = 3;

 //Left Hip Side to Side Motion Signals
 int LHipSide_Enable = 4;
 int LHipSide_A = 5;
 int LHipSide_B = 6;
 int LHipSide_HLFB = 7;

 //Left Hip Forward and Back Motion Signals
 int LHipFB_Enable = 8;
 int LHipFB_A = 9;
 int LHipFB_B = 10;
 int LHipFB_HLFB = 11;

// //Left Hip Rotation Signals
// int LHipRot_Enable = 27;
// int LHipRot_A = 26;
// int LHipRot_B = 25;
// int LHipRot_HLFB = 24;

 //Left Ankle Side to Side (twist) Motion
 int LAnkleTilt_Enable = 14;
 int LAnkleTilt_A = 21;
 int LAnkleTilt_B = 22;
 int LAnkleTilt_HLFB = 23;

 //Left Ankle Up and Down Motion
 int LAnkleFB_Enable = 12;
 int LAnkleFB_A = 15;     
 int LAnkleFB_B = 20;
 int LAnkleFB_HLFB = 19;

void setup() {
  // Set all High Level Feedback (HLFB) signals as Teensy inputs
  pinMode(LKneeFB_HLFB, INPUT_PULLUP);
  pinMode(LHipSide_HLFB, INPUT_PULLUP);
  pinMode(LHipFB_HLFB, INPUT_PULLUP);
//  pinMode(LHipRot_HLFB, INPUT_PULLUP);
  pinMode(LAnkleTilt_HLFB, INPUT_PULLUP);
  pinMode(LAnkleFB_HLFB, INPUT_PULLUP);

  // Set all motor control signals as Teensy outputs
  pinMode(LKneeFB_Enable, OUTPUT);
  pinMode(LKneeFB_A, OUTPUT);
  pinMode(LKneeFB_B, OUTPUT);

  pinMode(LHipSide_Enable, OUTPUT);
  pinMode(LHipSide_A, OUTPUT);
  pinMode(LHipSide_B, OUTPUT);

  pinMode(LHipFB_Enable, OUTPUT);
  pinMode(LHipFB_A, OUTPUT);
  pinMode(LHipFB_B, OUTPUT);

//  pinMode(LHipRot_Enable, OUTPUT);
//  pinMode(LHipRot_A, OUTPUT);
//  pinMode(LHipRot_B, OUTPUT);

  pinMode(LAnkleTilt_Enable, OUTPUT);
  pinMode(LAnkleTilt_A, OUTPUT);
  pinMode(LAnkleTilt_B, OUTPUT);

  pinMode(LAnkleFB_Enable, OUTPUT);
  pinMode(LAnkleFB_A, OUTPUT);
  pinMode(LAnkleFB_B, OUTPUT);

  //This section controls the speed of the motors
  analogWriteResolution(16); //Sets resolution of PWM signals, for more info https://www.pjrc.com/teensy/td_pulse.html
  analogWriteFrequency(LKneeFB_B, 14000); // Sets frquency for timer FTM1, pins 3 & 4 (Knee)
//  analogWriteFrequency(LHipRot_B, 14000); // Sets frequency for timer FTM2, pins 25 & 32 (Hip Rotation)
  analogWriteFrequency(LAnkleFB_B, 14000); // Sets frequency for timer FTM0, pins 5, 6, 9, 10, 20, 21, 22, 23 (Hip Side, Hip FB, Ankle FB, Ankle Tilt)
}

void loop() {
//Set directions for joints
digitalWrite(LAnkleTilt_A, HIGH); //Set foot to tilt left side up
digitalWrite(LAnkleFB_A, HIGH); //Set foot to flex up
digitalWrite(LHipSide_A, LOW); //Set hip to push outward
digitalWrite(LKneeFB_A, HIGH); //Set knee to bend outward
digitalWrite(LHipFB_A, LOW);  //Set hip to bend outward
//digitalWrite(LHipRot_A, LOW); 
delay(10);

//Move each joint individually, moving up the leg

digitalWrite(LAnkleTilt_Enable, HIGH); //Enable motor
analogWrite(LAnkleTilt_B, 10000);      //Assert motor to move
delay(1000);                           //Keep motor moving for 1 second
analogWrite(LAnkleTilt_B, 0);          //Stop motor motion
digitalWrite(LAnkleTilt_Enable, LOW);  //Disable motor

digitalWrite(LAnkleFB_Enable, HIGH);  
analogWrite(LAnkleFB_B, 10000);
delay(1000);
analogWrite(LAnkleFB_B, 0);
digitalWrite(LAnkleFB_Enable, LOW);

digitalWrite(LKneeFB_Enable, HIGH);
analogWrite(LKneeFB_B, 10000);
delay(1000);
analogWrite(LKneeFB_B, 0);
digitalWrite(LKneeFB_Enable, LOW);

digitalWrite(LHipFB_Enable, HIGH);
analogWrite(LHipFB_B, 10000);
delay(1000);
analogWrite(LHipFB_B, 0);
digitalWrite(LHipFB_Enable, LOW);

digitalWrite(LHipSide_Enable, HIGH);
analogWrite(LHipSide_B, 10000);
delay(500);
analogWrite(LHipSide_B, 0);
digitalWrite(LHipSide_Enable, LOW);

//digitalWrite(LHipRot_Enable, HIGH);
//analogWrite(LHipRot_B, 10000);
//delay(700);
//analogWrite(LHipRot_B, 0);
//digitalWrite(LHipRot_Enable, LOW);

//Switch direction signals for joints
digitalWrite(LAnkleTilt_A, LOW);
digitalWrite(LAnkleFB_A, LOW);
digitalWrite(LKneeFB_A, LOW);
digitalWrite(LHipFB_A, HIGH);
digitalWrite(LHipSide_A, HIGH);
//digitalWrite(LHipRot_A, HIGH);
delay(1000);   // Delay allows for time to adjust if something is non-operational

//Move each motor individually to reset the leg to initial position

digitalWrite(LAnkleTilt_Enable, HIGH);
analogWrite(LAnkleTilt_B, 10000);
delay(1000);
analogWrite(LAnkleTilt_B, 0);
digitalWrite(LAnkleTilt_Enable, LOW);

digitalWrite(LAnkleFB_Enable, HIGH);
analogWrite(LAnkleFB_B, 10000);
delay(1000);
analogWrite(LAnkleFB_B, 0);
digitalWrite(LAnkleFB_Enable, LOW);

digitalWrite(LKneeFB_Enable, HIGH);
analogWrite(LKneeFB_B, 10000);
delay(1000);
analogWrite(LKneeFB_B, 0);
digitalWrite(LKneeFB_Enable, LOW);

digitalWrite(LHipFB_Enable, HIGH);
analogWrite(LHipFB_B, 10000);
delay(1000);
analogWrite(LHipFB_B, 0);
digitalWrite(LHipFB_Enable, LOW);

digitalWrite(LHipSide_Enable, HIGH);
analogWrite(LHipSide_B, 10000);
delay(500);
analogWrite(LHipSide_B, 0);
digitalWrite(LHipSide_Enable, LOW);

//digitalWrite(LHipRot_Enable, HIGH);
//analogWrite(LHipRot_B, 10000);
//delay(700);
//analogWrite(LHipRot_B, 0);
//digitalWrite(LHipRot_Enable, LOW);

delay(5000);    //Delays loop for adjustments if something is non-operational or if the program only needs one run
}
