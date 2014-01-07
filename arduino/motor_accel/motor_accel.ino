#ifndef TWI_FREQ
#define TWI_FREQ 10000L
#endif

#include <Tlc5940.h>
#include <AcceleroMMA7361.h>
#include <Wire.h>

unsigned char test[3] ={1,2,3};
int axis[3];
#define SLAVE_ADDRESS 0x03
int command = 0;
int param[3];
int value;
int newcommand = LOW;
int pidState = LOW;
AcceleroMMA7361 accelero;
int LED = 2;
int limit = 0;
int dump;
unsigned long time, now;
int eps = 3;
float kp = 50;
float ki = 0;
float kd = 0;
int dt = 300;
int debug = false;
class pid{
  public:
  int input = 0; 
  int output = 0; 
  int setpoint =0;
  int error = 0; 
  int perror = 0; 
  int integral = 0; 
  int derivative = 0;
  void update();
};
void pid::update(){

  error = setpoint - input;
  if (abs(error)<eps) {error=0;}
  integral = integral + error*dt;
  derivative = (error - perror)/dt;
  output = (kp*error + ki*integral + kd*derivative);
  if (output > 4095) {output = 4095;}
  if (output < -4095) {output = -4095;}
  perror = error;
}
pid front, left, right;

int channels[6] = {0,0,0,0,0,0};
int down[6] = {0,4095,0,4095,0,4095};
int up[6] = {4095,0,4095,0,4095,0};
int flat[6] = {0,0,0,0,0,0};

void setup()
{

  Serial.begin(9600);                             // start serial for output
  Wire.begin(SLAVE_ADDRESS);                      // initialize i2c as slave
  Wire.onRequest(sendData);                       // register event
  Wire.onReceive(receiveData);                    // register event 
  accelero.begin(6,7,5,8,A0,A1,A2);               //(sleepPin, selfTestPin, zeroGPin, gSelectPin, xPin, yPin, zPin)
  accelero.setARefVoltage(3.3);                   //sets the AREF voltage to 3.3V
  accelero.setSensitivity(LOW);                  //sets the sensitivity to +/-6G or +/-1.5G (LOW/HIGH)
  accelero.calibrate();  
  pinMode(LED,OUTPUT);

  Tlc.init();

  axis[0] = accelero.getXAccel();
  axis[1] = accelero.getYAccel();
  axis[2] = accelero.getZAccel();
  Serial.print("Initial Axis: x: ");
  Serial.print(axis[0]);
  Serial.print(" y: ");
  Serial.print(axis[1]);
  Serial.print(" z: ");
  Serial.print(axis[2]);
  Serial.println("\n");
  front.setpoint = axis[0];
  right.setpoint = axis[1];
  left.setpoint = -axis[1];
  Serial.println("\r\nReady!");
}



// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void sendData()
{
  if (command==0){
    Wire.write(axis[param[0]]);  
  } else if (command == 10) {
    if(param[0]==0){
      Wire.write(int(kp));
    } else if(param[0]==1){
      Wire.write(int(ki));
    } else {
      Wire.write(int(kd));
    }
  } else if(command==99){
      Wire.write(test,3);
  } else {
    Wire.write(NULL);
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveData(int howMany)
{
  int i;
  // commands:
  // 0    read x, y or z accellerometer - followed by selection parameter 0-3
  // 1    set front set point - followed by value
  // 2    set left/right set point - followed by value
  // 3    set eps - followed by value
  // 4    all motors up - followed by duration in ms
  // 5    all motors down - followed by duration in ms
  // 6    turn PID on/off - followed by 1/0 for on/off
  // 7    set Kp - followed by value
  // 8    set Ki - followed by value
  // 9    set Kd - followed by value
  // 10   read Kp, Ki, or Kd - followed by selection parameter 0-3
  // 11   recallibrate
  // 12   set dt - followed by value
  // 13   set debug
  newcommand = HIGH;
  for(i=0; i<howMany; i++){
    if(i==0){
      command = Wire.read();
    } else if (i<4) {
      param[i-1] = Wire.read();    
    } else {
      dump = Wire.read(); 
    }
  }
}

void setchannels(unsigned int wait){
  Tlc.clear();

  for(int i=0; i<6; i++){
    Tlc.set(i,channels[i]);
  }
  Tlc.update();
  delay(wait);
  for(int i=0; i<6; i++){
    channels[i]=flat[i];
    Tlc.set(i,channels[i]);
  }
  Tlc.clear();
  Tlc.update();  
  delay(wait);

}

void loop()
{
  time = millis();
  int i;


  if (newcommand==HIGH){
    switch (command){
    case 1:
      if(param[0]>127){param[0]-=256;}
      front.setpoint = param[0];
    break;
    case 2:
      if(param[0]>127){param[0]-=256;}
      right.setpoint = param[0];
      left.setpoint = -param[0];      
    break;
    case 3:
      eps = param[0];            
    break;
    case 4:
      for(i=0;i<6;i+=2){
        channels[i]=up[i];
      }
      setchannels(param[0]);
      for(i=0;i<6;i+=2){
        channels[i]=flat[i];
      }      
      setchannels(param[0]);
    break;
    case 5:
      for(i=0;i<6;i++){
        channels[i]=down[i];    
      }
      setchannels(param[0]);  
      for(i=0;i<6;i+=2){
        channels[i]=flat[i];
      }        
      setchannels(param[0]);
    break;
    case 6:
      if(param[0] == 0){
        pidState = LOW;
      } else {
        pidState = HIGH;
      }
    break;
    case 7:
      kp = param[0];     
    break;
    case 8:
      ki = float(param[0])/100;
    break;
    case 9:
      kd = float(param[0])/10;
    break;
    case 11:
      accelero.calibrate();
      front.setpoint = axis[0];
      right.setpoint = axis[1]/2;
      left.setpoint = -axis[1]/2;      
    break;
    case 12:
      dt = param[0];
    break;
    case 13:
      debug = !debug;
    break;    
    case 99:
      
    break;
    default:
    break;
    }
    
  }
  if (newcommand==HIGH && debug==true){

    newcommand = LOW;
    Serial.println("\n----------------------------\n");
    Serial.print("MA command: ");
    Serial.print(command);
    Serial.print(" : ");
    Serial.print(param[0]);
    Serial.print(" - ");
    Serial.print(param[1]);
    Serial.print(" - ");
    Serial.print(param[2]);
    Serial.println("\n");
    Serial.print("Axis: x: ");
    Serial.print(axis[0]);
    Serial.print(" y: ");
    Serial.print(axis[1]);
    Serial.print(" z: ");
    Serial.print(axis[2]);    
    Serial.println("\n");
    Serial.print("PID: kp: ");
    Serial.print(kp);    
    Serial.print(" ki: ");
    Serial.print(ki);    
    Serial.print(" kd: ");
    Serial.print(kd);    
    Serial.println("\n");
    Serial.print("pidState: ");
    Serial.print(pidState);
    Serial.println("\n");
    Serial.print("Inputs: F: ");
    Serial.print(front.input);
    Serial.print(" R: ");
    Serial.print(right.input);
    Serial.print(" L: ");
    Serial.print(left.input);
    Serial.println("\n");
    Serial.print("Outputs: F: ");
    Serial.print(front.output);
    Serial.print(" R: ");
    Serial.print(right.output);
    Serial.print(" L: ");
    Serial.print(left.output);
    Serial.println("\n");
    Serial.print("Setpoints: F: ");
    Serial.print(front.setpoint);
    Serial.print(" R: ");
    Serial.print(right.setpoint);
    Serial.print(" L: ");
    Serial.print(left.setpoint);
    Serial.println("\n");
    Serial.print(" eps: ");
    Serial.print(eps);
    Serial.println("\n");    
    Serial.print("Error: F: ");
    Serial.print(front.error);
    Serial.print(" R: ");
    Serial.print(right.error);
    Serial.print(" L: ");
    Serial.print(left.error);
    Serial.println("\n");
    Serial.print("Channels: ");
    for(i=0;i<6;i++){
      Serial.print(i);
      Serial.print(": ");
      Serial.print(channels[i]);
      Serial.print(" ");
    }
    Serial.println("\n");
    Serial.print("dt: ");
    Serial.print(dt);
    Serial.println("\n");
    
  }   
  command = NULL;
  axis[0] = accelero.getXAccel();
  axis[1] = accelero.getYAccel();
  axis[2] = accelero.getZAccel();  
  front.input = axis[0];
  right.input = axis[1];
  left.input = -axis[1];
  front.update();
  right.update();
  left.update();
  if (pidState == HIGH){
    for(i=0; i<6; i++){
      channels[i]=flat[i];
    }
    if(front.output>0){
      channels[0]+=abs(front.output);
    } else {
      channels[1]+=abs(front.output/2);
    }

    if(left.output>0){
      channels[2]+=abs(left.output);
    } else {
      channels[3]+=abs(left.output/2);
    }

    if(right.output>0){
      channels[4]+=abs(right.output);
    } else {
      channels[5]+=abs(right.output/2);
    }    
    setchannels(50);
  } 
    delay(50);
/*
  delay(100);  //wait for 100ms
  limit ++;
  if(limit==5){
    digitalWrite(LED,HIGH);
  } else if (limit>=10) {
    digitalWrite(LED,LOW);
    limit=0;
  }
*/
  dt=millis() - time;
}

