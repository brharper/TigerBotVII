 //Left Ankle Up and Down Motion
 //Code says 12, 15, 20, and 19, PCB says 24, 26, 25, 27
 int LAnkleFB_Enable = 24;
 int LAnkleFB_A = 26;     
 int LAnkleFB_B = 25;
 int LAnkleFB_HLFB = 27;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    pinMode(LAnkleFB_HLFB, INPUT_PULLUP);
      pinMode(LAnkleFB_Enable, OUTPUT);
  pinMode(LAnkleFB_A, OUTPUT);
  pinMode(LAnkleFB_B, OUTPUT);
    analogWriteResolution(16);
    analogWriteFrequency(LAnkleFB_B, 14000);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Starting commands to flex up");
  digitalWrite(LAnkleFB_A, HIGH); //Set foot to flex up
digitalWrite(LAnkleFB_Enable, HIGH);  
analogWrite(LAnkleFB_B, 10000);
delay(5000);
analogWrite(LAnkleFB_B, 0);
digitalWrite(LAnkleFB_Enable, LOW);
Serial.println("Done with commands");
delay(1000);

Serial.println("Starting commands to flex down");
  digitalWrite(LAnkleFB_A, LOW); //Set foot to flex up
digitalWrite(LAnkleFB_Enable, HIGH);  
analogWrite(LAnkleFB_B, 10000);
delay(5000);
analogWrite(LAnkleFB_B, 0);
digitalWrite(LAnkleFB_Enable, LOW);
Serial.println("Done with commands");
delay(1000);
}
