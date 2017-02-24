
#include <SPI.h>

#define CS 3 //Chip or Slave select 

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];

float deg = 0.00;

void setup()
{
  pinMode(CS,OUTPUT);//Slave Select
  digitalWrite(CS,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  Serial.begin(115200);
  Serial.println("starting");
  Serial.flush();
  delay(3000);
  SPI.end();

}
uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
   
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(CS,LOW);     //select spi device
   msg_temp = SPI.transfer(msg);    //send and recieve
   digitalWrite(CS,HIGH);    //deselect spi device
   Serial.println(msg_temp);
   return(msg_temp);      //return recieved byte
}

void loop()
{ 
   uint8_t recieved = 0xA5;    //just a temp vairable
   ABSposition = 0;    //reset position vairable
   
   SPI.begin();    //start transmition
   digitalWrite(CS,LOW);
   
   SPI_T(0x10);   //issue read command
   
   recieved = SPI_T(0x00);    //issue NOP to check if encoder is ready to send
   Serial.println("In loop");
   while (recieved != 0x10)    //loop while encoder is not ready to send
   //for (int i=0; i<50; i++)
   {
     recieved = SPI_T(0x00);    //cleck again if encoder is still working 
     delay(2);    //wait a bit
//     if (recieved==0x10){
//      i=50;
//      Serial.println("Got 0x10");
//     }
   }

   Serial.println("Out of loop");
   temp[0] = SPI_T(0x00);    //Recieve MSB
   temp[1] = SPI_T(0x00);    // recieve LSB
   
   digitalWrite(CS,HIGH);  //just to make sure   
   SPI.end();    //end transmition
   
   temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
   ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    
   //if (ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
   //{
     ABSposition_last = ABSposition;    //set last position to current position
     deg = ABSposition;
     deg = deg * 0.08789;    // aprox 360/4096
     Serial.print("Degree: ");
     Serial.println(deg);     //send position in degrees
   //}   

   delay(1000);    //wait a bit till next check

}
