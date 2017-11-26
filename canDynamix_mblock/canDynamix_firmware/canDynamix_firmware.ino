/*************************************************************************
* File Name          : canDynamix_firmware.ino
* Author             : Kyung Man Shin
* Updated            : Kyung Man Shin
* Version            : V1.0
* Date               : 10/06/2017
* Description        : Firmware for canDynamix with Scratch.  
* License            : CC-BY-SA 3.0
**************************************************************************/

#include <Servo.h>
#include "motor.h"

   Servo servo;
   Servo servo1;
   Servo servo2;
   Servo servo3;
   Servo servo4;
 
boolean isAvailable = false;
boolean isBluetooth = false;

 int analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};

typedef struct MeModule
{
    int device;
    int port;
    int slot;
    int pin;
    int index;
    float values[3];
} MeModule;

union{
    byte byteVal[4];
    float floatVal;
    long longVal;
}val;

union{
  byte byteVal[8];
  double doubleVal;
}valDouble;

union{
  byte byteVal[2];
  short shortVal;
}valShort;


int len = 52;
char buffer[52];
char bufferBt[52];
byte index = 0;
byte dataLen;
byte modulesLen=0;
boolean isStart = false;
unsigned char irRead;
char serialRead;
#define VERSION 0
#define ULTRASONIC_SENSOR 1
#define TEMPERATURE_SENSOR 2
#define LIGHT_SENSOR 3
#define POTENTIONMETER 4
#define JOYSTICK 5
#define GYRO 6
#define SOUND_SENSOR 7
#define RGBLED 8
#define SEVSEG 9
#define MOTOR 100
#define SERVO 11
//#define ENCODER 12
#define IR 13
#define PIRMOTION 15
#define INFRARED 16
#define LINEFOLLOWER 17
#define SHUTTER 20
#define LIMITSWITCH 21
#define BUTTON 22
#define DIGITAL 30
#define ANALOG 31
#define PWM 32
#define SERVO_PIN 33
#define TONE 34
#define PULSEIN 35
#define ULTRASONIC_ARDUINO 36
#define STEPPER 40
#define ENCODER 41
#define TIMER 50
#define L_SERVO 37
#define R_SERVO 38
#define BUZZER 39
#define LED 42

#define N20 100

#define GET 1
#define RUN 2
#define RESET 4
#define START 5
float angleServo = 90.0;
unsigned char prevc=0;
double lastTime = 0.0;
double currentTime = 0.0;

void setup() {
  motorBegin();
  pinMode(13,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(13,HIGH);
  delay(300);
  digitalWrite(13,LOW);
  Serial.begin(115200);


}

void loop() {


   readSerial();
    if(isAvailable){
    unsigned char c = serialRead&0xff;
    if(c==0x55&&isStart==false){
     if(prevc==0xff){
      index=1;
      isStart = true;
     }
    }else{
      prevc = c;
      if(isStart){
        if(index==2){
         dataLen = c; 
        }else if(index>2){
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
     index++;
     if(index>51){
      index=0; 
      isStart=false;
     }
     if(isStart&&dataLen==0&&index>3){ 
        isStart = false;
        parseData(); 
        index=0;
     }
  }
}

unsigned char readBuffer(int index){
 return isBluetooth?bufferBt[index]:buffer[index]; 
}
void writeBuffer(int index,unsigned char c){
 if(isBluetooth){
  bufferBt[index]=c;
 }else{
  buffer[index]=c;
 } 
}
void writeHead(){
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeEnd(){
 Serial.println(); 
 #if defined(__AVR_ATmega32U4__) 
   Serial1.println();
 #endif
}
void writeSerial(unsigned char c){
 Serial.write(c);
 #if defined(__AVR_ATmega32U4__) 
   Serial1.write(c);
 #endif
}
void readSerial(){
  isAvailable = false;
  if(Serial.available()>0){
    isAvailable = true;
    isBluetooth = false;
    serialRead = Serial.read();
  }
  #if defined(__AVR_ATmega32U4__) 
  if(Serial1.available()>0){
    isAvailable = true;
    isBluetooth = true;
    serialRead = Serial1.read();
  }
 #endif
}

/*
ff 55 len idx action device port  slot  data a
0  1  2   3   4      5      6     7     8
*/

void parseData(){
  isStart = false;
  int idx = readBuffer(3);
  int action = readBuffer(4);
  int device = readBuffer(5);
  switch(action){
    case GET:{
        writeHead();
        writeSerial(idx);
        readSensor(device);
        writeEnd();
     }
     break;
     case RUN:{
       runModule(device);
       callOK();
     }
      break;
      case RESET:
     break;
     case START:{
        //start
        callOK();
      }
     break;
  }
}
void callOK(){
    writeSerial(0xff);
    writeSerial(0x55);
    writeEnd();
}

void sendByte(char c){
  writeSerial(1);
  writeSerial(c);
}
void sendString(String s){
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int i=0;i<l;i++){
    writeSerial(s.charAt(i));
  }
}
void sendFloat(float value){ 
     writeSerial(0x2);
     val.floatVal = value;
     writeSerial(val.byteVal[0]);
     writeSerial(val.byteVal[1]);
     writeSerial(val.byteVal[2]);
     writeSerial(val.byteVal[3]);
}
void sendShort(double value){
     writeSerial(3);
     valShort.shortVal = value;
     writeSerial(valShort.byteVal[0]);
     writeSerial(valShort.byteVal[1]);
}
void sendDouble(double value){
     writeSerial(2);
     valDouble.doubleVal = value;
     writeSerial(valDouble.byteVal[0]);
     writeSerial(valDouble.byteVal[1]);
     writeSerial(valDouble.byteVal[2]);
     writeSerial(valDouble.byteVal[3]);
}
short readShort(int idx){
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}
float readFloat(int idx){
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}
void runModule(int device){
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa 
  int port = readBuffer(6);
  int pin = port;

//  if(device == L_SERVO){ servo1.attach(7); }
//  else if(device == R_SERVO){ servo2.attach(8); }
//  else { servo1.attach(100); servo2.attach(101); }

  switch(device){
   case MOTOR:{
     int leftSpeed =  readBuffer(6);
     int rightSpeed =  readBuffer(7);
     if(leftSpeed >= 200){ leftSpeed = -(256-leftSpeed); }
     if(rightSpeed >= 200){ rightSpeed = -(256-rightSpeed); }
      motorMoveSpeed(leftSpeed, rightSpeed);
   }
    break;
   
    case JOYSTICK:
    break;
   // case STEPPER: 
    //break;
    case ENCODER:
    break;
   case RGBLED:
   break;
   case SERVO:
   break;

  case LED:{

     int v = readBuffer(6);
     int v1 = readBuffer(7);
     digitalWrite(11,v);
     digitalWrite(10,v1);
  }
   break;
  
  case BUZZER:{
     int v = readBuffer(6);
     digitalWrite(13,v);
  }
   break;

   
   case SERVO_PIN:{
     int val =  readBuffer(6);
     int val1 =  readBuffer(7);
     if(val>=0&&val<=180&&val1>=0&&val1<=180){
        servo3.attach(4); 
        servo3.write(val); 
        servo4.attach(12); 
        servo4.write(val1); 
     }
       
   }
   break;
//   
//  case L_SERVO:{
//
//
//     int v =  readBuffer(7);
//     if(v>=0&&v<=180){
//        servo1.attach(7); 
//        servo1.write(v); 
//        
//    
//     }
//   }
//   break;
//
//  case R_SERVO:{
//
//     int v =  readBuffer(7);
//     if(v>=0&&v<=180){
//        servo2.attach(8); 
//        servo2.write(v); 
//     }
//   }
//   break;
//   
   case SEVSEG:
   break;
   case LIGHT_SENSOR:
   break;
   case SHUTTER:
   break;
   case DIGITAL:{
     pinMode(pin,OUTPUT);
     int v = readBuffer(7);
     digitalWrite(pin,v);
   }
   break;
   case PWM:{
     pinMode(pin,OUTPUT);
     int v = readBuffer(7);
     analogWrite(pin,v);
   }
   break;
   case TONE:{
     pinMode(pin,OUTPUT);
     int hz = readShort(7);
     int ms = readShort(9);
     if(ms>0){
       tone(pin, hz, ms); 
     }else{
       noTone(pin); 
     }
   }
   break;

   case TIMER:{
    lastTime = millis()/1000.0; 
   }
   break;
  }
}
void readSensor(int device){
  /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value=0.0;
  int port,slot,pin;
  port = readBuffer(6);
  pin = port;
  switch(device){
   case  ULTRASONIC_SENSOR:
   break;
   case  TEMPERATURE_SENSOR:
   break;
   case  LIGHT_SENSOR:
   case  SOUND_SENSOR:
   case  POTENTIONMETER:
   break;
   case  JOYSTICK:
   break;
   case  INFRARED:
   break;
   case  PIRMOTION:
   break;
   case  LINEFOLLOWER:
   break;
   case LIMITSWITCH:
   break;
   case  GYRO:
   break;
   case  VERSION:{
    // sendString(mVersion);
   }
   break;
   case  DIGITAL:{
     pinMode(pin,INPUT);
     sendFloat(digitalRead(pin));
   }
   break;
   case  ANALOG:{
     pin = analogs[pin];
     pinMode(pin,INPUT);
     sendFloat(analogRead(pin));
   }
   break;
   case  PULSEIN:
   break;
   case ULTRASONIC_ARDUINO:
   break;
   case TIMER:{
     sendFloat((float)currentTime);
   }
   break;
  }
}





