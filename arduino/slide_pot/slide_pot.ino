#ifndef TWI_FREQ
#define TWI_FREQ 10000L
#endif
#include <Wire.h>
#define SLAVE_ADDRESS 0x05
int Fpin = A0;  //Analogue pin for front slide pot
int Lpin = A1;  //Analogue pin for left slide pot
int Rpin = A2;  //Analogue pin for right slide pot
int pots[3];  //slide pot values front, left, right
int boop = LOW;
int command;
int value;
int LED = 2;
int limit = 0;
void setup()
{
  Wire.begin(SLAVE_ADDRESS);                // join i2c bus with address #5
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(9600);           // start serial for output  
  pinMode(LED,OUTPUT);
  analogReference(DEFAULT);
  Serial.println("\r\nReady!");
}


// callback for received data
void receiveData(int byteCount){

    while(Wire.available()) {
        command = Wire.read();
        boop = HIGH;

        if (command < 3 && command >= 0){
          value = pots[command];
        } else {
          value = NULL;
        }
     }
}

// callback for sending data
void sendData(){
  Wire.write(value);
}

void loop()
{
  pots[0] = analogRead(Fpin)/4;    
  pots[1] = analogRead(Lpin)/4;      
  pots[2] = analogRead(Rpin)/4;   

  if (boop==HIGH){
    boop = LOW;
    Serial.print("SP command: ");
    Serial.print(command);
    Serial.print(" : ");
    Serial.print(pots[0]);
    Serial.print(" - ");
    Serial.print(pots[1]);
    Serial.print(" - ");
    Serial.print(pots[2]);
    Serial.println("\n");
    Serial.println("\n");
  }    
  delay(100);  //wait for 100ms
  limit ++;
  if(limit==5){
    digitalWrite(LED,HIGH);
  } else if (limit>=10) {
    digitalWrite(LED,LOW);
    limit=0;
  }
}
