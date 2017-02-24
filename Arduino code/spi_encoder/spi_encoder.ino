//MOSI: White (6) to 11
//MISO: Green (4) to 12
//CLK: Yellow (5) to 13
//SS: Blue (3) to whichever

//Read command code from here: http://forum.arduino.cc/index.php?topic=158790.0
//About set_zero_point: When it works correctly, sets current position reading to zero. Appears to work when power isn't cycled afterwards.
// Cycling seems to add a random offset to the reading.
// Some issues may be caused by wiring issues, since SPI messages get messed up by moving encoder/wires
//2/22/17

#include <SPI.h>

#define CS 10 //Chip or Slave select 

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];

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
   //Serial.println(msg_temp);
   return(msg_temp);      //return recieved byte
}

void loop()
{ 
  Serial.println("Reading positions");
  uint8_t recieved = 0xA5;    //just a temp vairable
  for(int i= 0; i<100; i++){
   
   ABSposition = 0;    //reset position vairable
   
   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
   digitalWrite(CS,LOW);

   //Serial.println("Sending read command");
   //SPI_T(0x70);   //issue read command

   
   recieved = SPI_T(0x10);    //issue NOP to check if encoder is ready to send
   //Serial.println("In loop");
   while (recieved != 0x10)    //loop while encoder is not ready to send
   {
    /*Need to change 0x10 to 0x00- 0x10 means it's been repeatedly sending read commands. Shouldn't affect how it works, but 0x00 is how it's supposed to be:*/
     recieved = SPI_T(0x10);    //cleck again if encoder is still working 
     delay(2);    //wait a bit- not required
   }
   //Serial.println("Out of loop");
  //delay(1000);
   temp[0] = SPI_T(0x00);    //Recieve MSB
   temp[1] = SPI_T(0x00);    // recieve LSB
   
   digitalWrite(CS,HIGH);  //just to make sure   
   //SPI.end();    //end transmition
   SPI.endTransaction();
   
   temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
   ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    
   if (ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
   {
     ABSposition_last = ABSposition;    //set last position to current position
     deg = ABSposition;
     deg = deg * 0.08789;    // aprox 360/4096
     Serial.print("Degree: ");
     Serial.println(deg);     //send position in degrees
   }   

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
   
   recieved = SPI_T(0x70);    //issue NOP to check if encoder is ready to send
   //Serial.println("In loop");
   while (recieved != 0x80)    //loop while encoder is not ready to send
   {
     recieved = SPI_T(0x00);    //cleck again if encoder is still working 
     delay(20);    //wait a bit- not required
   }
   Serial.println("Zero-point done");
//   for (int j=0; j<10; j++) {
//    temp[0] = SPI_T(0x00);
//   }
   digitalWrite(CS,HIGH);  //just to make sure   
   //SPI.end();    //end transmition
   SPI.endTransaction();
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

