//MOSI: White (6) to 11
//MISO: Green (4) to 12
//CLK: Yellow (5) to 13
//SS: Blue (3) to whichever

//Read command code from here: http://forum.arduino.cc/index.php?topic=158790.0
//About set_zero_point: When it works correctly, sets current position reading to zero. Appears to work when power isn't cycled afterwards.
// Cycling seems to add a random offset to the reading.
// Some issues may be caused by wiring issues, since SPI messages get messed up by moving encoder/wires
//eg: not getting a 16 response while reading, getting only 37 degrees (17, 165) probably means power issue, check if encoder's drawing ~11mA or not
//2/22/17

//Times: reading takes 0.010s total (.002 for first command + .004 in loop + .004 til end)
//       zero point takes 0.012s total (.002 for first command, .010 in loop until done)
//Overload by sending 0x10 in loop: first reading takes .01, sequential readings take .006, zero-point takes .014
//Overload by sending 0x10 when reading pos data: kills zero-point

//Need to figure out timing of overloading, if going to use it
//3/9/17

#include <SPI.h>

#define CS 10 //Chip or Slave select 

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];
unsigned long start_time=0, time_start, time1, time2, time3, timez_start, timez1, timez2, timez3;

float deg = 0.00;

int state=0;

void setup()
{
  pinMode(CS,OUTPUT);//Slave Select
  digitalWrite(CS,HIGH);
  SPI.begin();

  Serial.begin(9600);
  
  Serial.flush();
  delay(3000);
  Serial.println("starting");
  start_time=millis();

}
uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(CS,LOW);     //select spi device
   // delay before and after SPI.transfer(msg) is required for correct communication
   delay(1);
   msg_temp = SPI.transfer(msg);    //send and recieve
   delay(1);
   digitalWrite(CS,HIGH);    //deselect spi device
   Serial.println(msg_temp);
   return(msg_temp);      //return recieved byte
   
}

void loop()
{ 
  //Serial.println("Reading positions");
  uint8_t recieved = 0xA5;    //just a temp vairable
  for(int i= 0; i<100; i++){
   
   ABSposition = 0;    //reset position vairable
   
   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
   digitalWrite(CS,LOW);

   //Serial.println("Sending read command");
   //SPI_T(0x70);   //issue read command

   time_start=micros();
   recieved = SPI_T(0x10);    //issue NOP to check if encoder is ready to send
   //Serial.println("In loop");
   time1=micros();
   while (recieved != 0x10)    //loop while encoder is not ready to send
   {
     recieved = SPI_T(0x00);    //check again if encoder is still working 
     //delay(2);    //wait a bit- not required
   }
   //Serial.println("Out of loop");
   time2=micros();
  //delay(1000);
   temp[0] = SPI_T(0x00);    //Recieve MSB
   temp[1] = SPI_T(0x00);    // recieve LSB
   
   digitalWrite(CS,HIGH);  //just to make sure   
   //SPI.end();    //end transmition
   SPI.endTransaction();
   time3=micros();
   temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
   ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    
//   if (ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
//   {
     ABSposition_last = ABSposition;    //set last position to current position
     deg = ABSposition;
     deg = deg * 0.08789;    // aprox 360/4096
    // Serial.print("Degree: ");
     Serial.print(deg);     //send position in degrees
     Serial.print(", time to send command=");
//     Serial.println(millis()-start_time);
//   }   
   Serial.print(time1-time_start);
   Serial.print(", time in loop=");
   Serial.print(time2-time1);
   Serial.print(", time to get last commands and end=");
   Serial.println(time3-time2);
   delay(10);    //wait a bit till next check
  }
  //Serial.println("Testing zero-point command, clearing queue");
//  for (int j=0; j<5; j++) {
//    temp[0] = SPI_T(0x00);
//  }
 // delay(1000);

   
   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
   digitalWrite(CS,LOW);

   Serial.println("Sending zero-point command (0x70)");
   //SPI_T(0x70);   //issue read command
   timez_start=micros();
   recieved = SPI_T(0x70);    //issue NOP to check if encoder is ready to send
   //Serial.println("In loop");
   timez1=micros();
   while (recieved != 0x80)    //loop while encoder is not ready to send
   {
     recieved = SPI_T(0x00);    //cleck again if encoder is still working 
     //delay(20);    //wait a bit- not required
   }
   timez2=micros();

//   for (int j=0; j<10; j++) {
//    temp[0] = SPI_T(0x00);
//   }
   digitalWrite(CS,HIGH);  //just to make sure   
   //SPI.end();    //end transmition
   SPI.endTransaction();
   timez3=micros();
      Serial.print("Zero-point done, time to send command=");
   Serial.print(timez1-timez_start);
   Serial.print(", time in loop=");
   Serial.print(timez2-timez1);
   Serial.print(", time to get last commands and end=");
   Serial.println(timez3-timez2);
  delay(2000);


}

//State machine function for reading the encoder, so don't wait on position ready response
/*void encoder_read() {
  
  uint8_t recieved = 0xA5;    //just a temp variable
  switch (state) {
    case 0:
      ABSposition = 0;    //reset position vairable
     
     SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); //May need begin and end within states?
  
     //Serial.println("Sending read command");
     received=SPI_T(0x10);   //issue read command
     if (received==16) state=2; //Position ready, go get position data
     else state=1;              //Wait for position ready
     break;

    case 1:
      received=SPI_T(0x00);   //issue NOP to check if encoder is ready to send
      if (received==16) state=2; //Position ready, go get position data
      else state=1;              //Wait for position ready
      break;

    case 2:
      temp[0] = SPI_T(0x00);    //Recieve MSB
      temp[1] = SPI_T(0x00);    // recieve LSB
      
      digitalWrite(CS,HIGH);  //just to make sure   
      //SPI.end();    //end transmition
      SPI.endTransaction();
      state=0;
      break;
  }

}*/

