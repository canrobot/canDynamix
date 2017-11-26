/* Authors: kyungman shin, Baram */

#include "mblock.h"
#include "motor.h"
#include "MedianFilter.h"
#include "src/mpu9250/MPU9250.h"
#include "src/mpu9250/MadgwickAHRS.h"

#include <Wire.h>

#define WHEEL_RADIUS                     0.0165          // meter
#define WHEEL_SEPARATION                 0.086           // meter (canDynamix n20 motor)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                     -2147483648      // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT  0
#define RIGHT 1



#define VELOCITY_CONSTANT_VALUE           135.1090523 // V = r * w = 1400 / 2 * PI * r 
                                                      // = 0.0165 * 0.229 * Goal RPM * 0.10472
                                                      // Goal Speed = V * 1263.632956882

#define DEG2RAD(x)                        (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                        (x * 57.2957795131)  // *180/PI
                                                      




void imuGetData(int16_t* a_x, int16_t* a_y, int16_t* a_z, int16_t* g_x, int16_t* g_y, int16_t* g_z);



/*******************************************************************************
* SoftwareTimer of canDynamix
*******************************************************************************/
static uint32_t tTime[4];



/*******************************************************************************
* Declaration for sonic
*******************************************************************************/
MedianFilter sonic_filter(16, 0);
#define sonic_trigger_pin  A0 //Trig pin
#define sonic_echo_pin     A1 //Echo pin
long     distance_mm;

/*******************************************************************************
* Declaration for MPU9250
*******************************************************************************/
MPU9250  mpu9250;
Madgwick filter;

float acc_cal[3];
float gyro_cal[3];

void imuCalibration(void);
void updateMPU9250(void);
void updateSonic(void);


/*******************************************************************************
* Declaration for mblock
*******************************************************************************/
void mblockRead(mblcok_packet_t *p_packet, uint8_t device);
void mblockRun(mblcok_packet_t *p_packet, uint8_t device);

void setup()
{
  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  
  pinMode(13, OUTPUT);
  pinMode(sonic_trigger_pin, OUTPUT); // Trigger is an output pin
  pinMode(sonic_echo_pin, INPUT);     // Echo is an input pin


  Wire.begin();  
  mpu9250.initialize();
  filter.begin(100);
  imuCalibration();

  mblockBegin(115200);
  mblockSetReadCallback(mblockRead);
  mblockSetRunCallback(mblockRun);  
}

void loop() {
  // 100Hz
  if ((millis()-tTime[1]) >= (1000 / 100))
  {
    tTime[1] = millis();
        
    updateMPU9250();
  }

  
  // 50Hz
  if ((millis()-tTime[2]) >= (1000 / 50))
  {
    tTime[2] = millis();
   
    updateSonic();
  }

  mblockUpdate();
}


/*******************************************************************************
* updateSonic
*******************************************************************************/
void updateSonic(void)
{
  uint32_t duration;
  

  digitalWrite(sonic_trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(sonic_trigger_pin, HIGH);    // Trigger pin to HIGH
  delayMicroseconds(10); // 10us high
  digitalWrite(sonic_trigger_pin, LOW);     // Trigger pin to HIGH

  // 최대 측정 시간을 7ms로 제한함 
  //
  duration = pulseIn(sonic_echo_pin, HIGH, 7000); // Waits for the echo pin to get high    
  if (duration == 0)
  {
    duration = 7000;
  }

  sonic_filter.in(duration);
  duration = sonic_filter.out();

  distance_mm = ((duration / 2.9) / 2);     // Actual calculation in mm
  //sonic_distance = (double)distance_mm / 1000.0;
  //sonic_msg.range = (double)distance_mm / 1000.0;
}


/*******************************************************************************
* updateMPU9250
*******************************************************************************/
void updateMPU9250(void)
{
  float aRes =    8.0 / 32768.0;
  float gRes = 2000.0 / 32768.0;


  int16_t ax, ay, az = 0;
  int16_t gx, gy, gz = 0;

  float acc[3];
  float gyro[3];


  imuGetData(&ax, &ay, &az, &gx, &gy, &gz);

  acc[0] = (float)((float)ax - acc_cal[0]) * aRes;
  acc[1] = (float)((float)ay - acc_cal[1]) * aRes;
  acc[2] = (float)az*aRes;

  gyro[0] = (float)((float)gx - gyro_cal[0]) * gRes;
  gyro[1] = (float)((float)gy - gyro_cal[1]) * gRes;
  gyro[2] = (float)((float)gz - gyro_cal[2]) * gRes;
  
  filter.updateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
}

void imuGetData(int16_t* a_x, int16_t* a_y, int16_t* a_z, int16_t* g_x, int16_t* g_y, int16_t* g_z)
{
  int16_t  a[3];
  int16_t  g[3];

  mpu9250.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);

  *a_x = -a[1];
  *a_y = a[0];
  *a_z = a[2];

  *g_x = -g[1];
  *g_y = g[0];
  *g_z = g[2];  
}

void imuCalibration(void)
{
	int      cal_int = 0;
  uint8_t  axis = 0;
  uint16_t cal_count = 1000;
  int16_t  a[3];
  int16_t  g[3];


  for(axis=0; axis<3; axis++)
  {
    acc_cal[axis]  = 0;
    gyro_cal[axis] = 0;
  }

  for (cal_int = 0; cal_int < cal_count; cal_int ++)
  {
    imuGetData(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
    
		for(axis=0; axis<3; axis++)
		{
      acc_cal[axis]  += (float)a[axis];
			gyro_cal[axis] += (float)g[axis];
		}
	}

	for(axis=0; axis<3; axis++)
	{
    acc_cal[axis]  /= (float)cal_count;
    gyro_cal[axis] /= (float)cal_count;
  }
  acc_cal[2] = 0;
}


void mblockRead(mblcok_packet_t *p_packet, uint8_t device)
{
  float value=0.0;
  int port,slot,pin;

  port = p_packet->port;
  pin = port;


  switch(device)
  {
    case  GYRO:
      break;
    case  VERSION:
      break;
    case  DIGITAL:
      pinMode(pin,INPUT);
      mblockSendFloat(digitalRead(pin));
      break;

    case  ANALOG:      
      //pinMode(pin,INPUT);
      if (pin == 0)
      {
        mblockSendFloat(distance_mm);
      }
      break;

    case  PULSEIN:
      break;

    case TIMER:
      mblockSendFloat((float)millis());
      break;
  }
}

void mblockRun(mblcok_packet_t *p_packet, uint8_t device)
{
  int port = p_packet->port;
  int pin  = port;
  int val;
  int val1;
  int v;
  int v1;
  int leftSpeed;
  int rightSpeed;
  int hz;
  int ms;


  switch(device)
  {
    case MOTOR:
      leftSpeed  = mblockReadBuffer(6);
      rightSpeed = mblockReadBuffer(7);
      if(leftSpeed >= 200) { leftSpeed  = -(256-leftSpeed);  }
      if(rightSpeed >= 200){ rightSpeed = -(256-rightSpeed); }
      motorMoveSpeed(leftSpeed, rightSpeed);
      break;
   
    case JOYSTICK:
      break;

    case ENCODER:
      break;

    case RGBLED:
      break;
    case SERVO:
      break;

    case LED:
      v  = mblockReadBuffer(6);
      v1 = mblockReadBuffer(7);
      digitalWrite(11,v);
      digitalWrite(10,v1);
      break;
  
    case BUZZER:
      v = mblockReadBuffer(6);
      digitalWrite(13,v);
      break;

   
    case SERVO_PIN:
      val  =  mblockReadBuffer(6);
      val1 =  mblockReadBuffer(7);
      /*
      if(val>=0&&val<=180&&val1>=0&&val1<=180)
      {
        servo3.attach(4); 
        servo3.write(val); 
        servo4.attach(12); 
        servo4.write(val1); 
      } 
      */      
      break;

    case DIGITAL:
      pinMode(pin,OUTPUT);
      v = mblockReadBuffer(7);
      digitalWrite(pin,v);
      break;

    case PWM:
      pinMode(pin,OUTPUT);
      v = mblockReadBuffer(7);
      analogWrite(pin,v);
      break;

    case TONE:
      pinMode(pin,OUTPUT);
      hz = mblockReadShort(7);
      ms = mblockReadShort(9);
      if(ms>0)
      {
        tone(pin, hz, ms); 
      }
      else
      {
       noTone(pin); 
      }
      break;

    case TIMER:
      break;
  }
}