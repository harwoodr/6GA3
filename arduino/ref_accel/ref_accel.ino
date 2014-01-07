#ifndef TWI_FREQ
#define TWI_FREQ 10000L
#endif
#include <Wire.h>
#include <AcceleroMMA7361.h>
AcceleroMMA7361 accelero;
int axis[3];
#define SLAVE_ADDRESS 0x04
int command = 0;
int value;
int boop;
int LED = 2;
void setup() {
    Serial.begin(9600);         // start serial for output
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);
  pinMode(LED,OUTPUT);
    // define callbacks for i2c communicatiorea
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
  accelero.begin(6,7,5,8,A0,A1,A2);                //(sleepPin, selfTestPin, zeroGPin, gSelectPin, xPin, yPin, zPin)
  accelero.setARefVoltage(3.3);                    //sets the AREF voltage to 3.3V
  accelero.setSensitivity(HIGH);                   //sets the sensitivity to +/-6G
  accelero.calibrate();  
    Serial.println("\r\nReady!");
}

void loop() {

    axis[0] = int(accelero.getXAccel());
    axis[1] = int(accelero.getYAccel());
    axis[2] = int(accelero.getZAccel()); 
   
    if (boop==HIGH){
      boop = LOW;
      Serial.print("RA command: ");
      Serial.print(command);
      Serial.print(" : ");
      Serial.print(axis[0]);
      Serial.print(" - ");
      Serial.print(axis[1]);
      Serial.print(" - ");
      Serial.print(axis[2]);
      Serial.println("\n");
    }  
  digitalWrite(LED,HIGH);
  delay(500);  //wait for 100ms
  digitalWrite(LED,LOW);
  delay(500);
}

// callback for received data
void receiveData(int byteCount){

    while(Wire.available()) {
        command = Wire.read();
        boop = HIGH;

        if (command < 3 && command >= 0){
          value = axis[command];
        } else {
          value = NULL;
        }
     }
}

// callback for sending data
void sendData(){
    Wire.write(value);

}



