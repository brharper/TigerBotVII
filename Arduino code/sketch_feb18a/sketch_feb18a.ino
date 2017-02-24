//MOSI: White (6) to 11
//MISO: Green (4) to 12
//CLK: Yellow (5) to 13
//SS: Blue (3) to whichever
//This code works

#include <SPI.h>

#define CS 10 //Chip or Slave select 

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];

float deg = 0.00;

void setup()
{
  pinMode(CS,OUTPUT);//Slave Select
  digitalWrite(CS,HIGH);
  SPI.begin();
//  SPI.setBitOrder(MSBFIRST);
//  SPI.setDataMode(SPI_MODE0);
  //SPI.setClockDivider(SPI_CLOCK_DIV128);
  Serial.begin(9600);
  
  Serial.flush();
  delay(3000);
  Serial.println("starting");
  //SPI.end();

}
uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(CS,LOW);     //select spi device
   // delay before and after SPI.transfer(msg) is required for correct communication
   delay(5);
   msg_temp = SPI.transfer(msg);    //send and recieve
   delay(5);
   digitalWrite(CS,HIGH);    //deselect spi device
   Serial.println(msg_temp);
   return(msg_temp);      //return recieved byte
}

void loop()
{ 
   uint8_t recieved = 0xA5;    //just a temp vairable
   ABSposition = 0;    //reset position vairable
   
   //SPI.begin();    //start transmition
   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
   digitalWrite(CS,LOW);

   Serial.println("Sending read command");
   SPI_T(0x10);   //issue read command
   
   recieved = SPI_T(0x00);    //issue NOP to check if encoder is ready to send
   //Serial.println("In loop");
   while (recieved != 0x10)    //loop while encoder is not ready to send
   {
     recieved = SPI_T(0x10);    //cleck again if encoder is still working 
     delay(20);    //wait a bit- not required
   }
   Serial.println("Out of loop");
//   for (int i=0; i<10; i++) {
//    SPI_T(0x00);
//   }
   temp[0] = SPI_T(0x00);    //Recieve MSB
   temp[1] = SPI_T(0x00);    // recieve LSB
   
   digitalWrite(CS,HIGH);  //just to make sure   
   //SPI.end();    //end transmition
   SPI.endTransaction();
   
   temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
   ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    
//   if (ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
//   {
     ABSposition_last = ABSposition;    //set last position to current position
     deg = ABSposition;
     deg = deg * 0.08789;    // aprox 360/4096
     Serial.print("Degree: ");
     Serial.println(deg);     //send position in degrees
//   }   

   delay(1000);    //wait a bit till next check

}
