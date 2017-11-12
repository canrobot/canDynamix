/* Authors: kyungman shin */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <candynamix_msgs/sensor.h>

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
                                                      
// Limit values 
#define LIMIT_X_MAX_VELOCITY              30
#define MAX_LINEAR_VELOCITY               0.22   // m/s
#define MAX_ANGULAR_VELOCITY              2.84 // rad/s




// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

void controlMotorSpeed(void);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);


/*******************************************************************************
* Publisher
*******************************************************************************/
candynamix_msgs::sensor sensor_msg;
ros::Publisher sensor_pub("/canDynamix/sensor", &sensor_msg);



/*******************************************************************************
* SoftwareTimer of canDynamix
*******************************************************************************/
static uint32_t tTime[4];


/*******************************************************************************
* Declaration for motor
*******************************************************************************/
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;


/*******************************************************************************
* Declaration for sonic
*******************************************************************************/
MedianFilter sonic_filter(16, 0);
#define sonic_trigger_pin  A0 //Trig pin
#define sonic_echo_pin     A1 //Echo pin


/*******************************************************************************
* Declaration for MPU9250
*******************************************************************************/
MPU9250  mpu9250;
Madgwick filter;

float acc_cal[3];
float gyro_cal[3];



void setup() {

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


  nh.getHardware()->setBaud(115200);
  nh.initNode();  
  nh.subscribe(cmd_vel_sub);
  nh.advertise(sensor_pub);
}

void loop() {
  static int32_t seq_ctr = 0;


  // 30Hz
  if ((millis()-tTime[0]) >= (1000 / 30))
  {
    controlMotorSpeed();
    tTime[0] = millis();
  }

  
  // 100Hz
  if ((millis()-tTime[1]) >= (1000 / 100))
  {
    tTime[1] = millis();

    sensor_msg.left_count  = motorGetCounter(L_MOTOR);
    sensor_msg.right_count = motorGetCounter(R_MOTOR);

        
    updateMPU9250();


    sensor_pub.publish(&sensor_msg);
  }

  
  // 50Hz
  if ((millis()-tTime[2]) >= (1000 / 50))
  {
    tTime[2] = millis();
   
    updateSonic();
  }
  

  nh.spinOnce();
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  digitalWrite(13, !digitalRead(13));
}


/*******************************************************************************
* Control motor speed
*******************************************************************************/
void controlMotorSpeed(void)
{
  double wheel_speed_cmd[2];
  double lin_vel_left;
  double lin_vel_right;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);

  lin_vel_left  = constrain(wheel_speed_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  lin_vel_right = constrain(wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  
  motorMoveSpeed((int32_t)lin_vel_left, (int32_t)lin_vel_right);
}

/*******************************************************************************
* updateSonic
*******************************************************************************/
void updateSonic(void)
{
  uint32_t duration;
  long     distance_mm;

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

  sensor_msg.sonic_distance_time = duration;
}


/*******************************************************************************
* updateMPU9250
*******************************************************************************/
void updateMPU9250(void)
{
  float aRes =   2.0 / 32768.0;
  float gRes = 250.0 / 32768.0;

  int16_t ax, ay, az = 0;
  int16_t gx, gy, gz = 0;

  float acc[3];
  float gyro[3];


  imuGetData(&ax, &ay, &az, &gx, &gy, &gz);

  #if 1
  acc[0] = (float)((float)ax - acc_cal[0]) * aRes;
  acc[1] = (float)((float)ay - acc_cal[1]) * aRes;
  acc[2] = (float)az*aRes;

  gyro[0] = (float)((float)gx - gyro_cal[0]) * gRes;
  gyro[1] = (float)((float)gy - gyro_cal[1]) * gRes;
  gyro[2] = (float)((float)gz - gyro_cal[2]) * gRes;
  #else
  acc[0] = (float)ax * aRes;
  acc[1] = (float)ay * aRes;
  acc[2] = (float)az * aRes;

  gyro[0] = (float)gx * gRes;
  gyro[1] = (float)gy * gRes;
  gyro[2] = (float)gz * gRes;
  #endif
  
  filter.updateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

  //sensor_msg.qw = filter.getRoll();
  //sensor_msg.qx = filter.getPitch();
  //sensor_msg.qy = filter.getYaw();

  sensor_msg.ax = ax;
  sensor_msg.ay = ay;
  sensor_msg.az = az;
  sensor_msg.gx = gx;
  sensor_msg.gy = gy;
  sensor_msg.gz = gz;

  sensor_msg.qw = filter.q0;
  sensor_msg.qx = filter.q1;
  sensor_msg.qy = filter.q2;
  sensor_msg.qz = filter.q3;
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
