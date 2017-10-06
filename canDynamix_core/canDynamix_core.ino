
/* Authors: kyungman shin */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>

#include "motor.h"

#define WHEEL_RADIUS                     0.033           // meter
//#define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define WHEEL_SEPARATION                 0.100           // meter (canDynamix n20 motor)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT  0
#define RIGHT 1

#define CONTROL_MOTOR_SPEED_PERIOD       30   //hz

#define VELOCITY_CONSTANT_VALUE          1263.632956882  // V = r * w = r * RPM * 0.10472
                                                         //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                         // Goal RPM = V * 1263.632956882

// Limit values (n20 encoder motor)
#define LIMIT_X_MAX_VELOCITY            255

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
std_msgs::String str_msg;
ros::Publisher canDynamix_pub("/canDynamix/state_msg", &str_msg);

std_msgs::Int64 int64_left_msg;
ros::Publisher leftPwm_pub("/canDynamix/left_pwm", &int64_left_msg);

std_msgs::Int64 int64_right_msg;
ros::Publisher rightPwm_pub("/canDynamix/right_pwm", &int64_right_msg);


/*******************************************************************************
* SoftwareTimer of canDynamix
*******************************************************************************/
static uint32_t tTime[4];


/*******************************************************************************
* Declaration for motor
*******************************************************************************/
bool init_encoder_[2]  = {false, false};
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

int count_left = 0;
int count_right = 0;


void setup() {

  Serial.begin(9600);
  pinMode(7,INPUT);
  pinMode(8,INPUT);

  nh.initNode();

  nh.subscribe(cmd_vel_sub);

  nh.advertise(canDynamix_pub);
  nh.advertise(leftPwm_pub);
  nh.advertise(rightPwm_pub);


}

void loop() {
  
 if ((millis()-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    controlMotorSpeed();
    tTime[0] = millis();
  }

 int left_out = digitalRead(2);
 int right_out = digitalRead(4);

if (left_out  == 1)  count_left  = count_left+1;
if (right_out == 1)  count_right = count_right+1;

//Serial.print("[left]  "); Serial.print(count_left);
//Serial.print("  [right] "); Serial.println(count_right);

 // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;
}


/*******************************************************************************
* Control motor speed
*******************************************************************************/
void controlMotorSpeed(void)
{
  //bool dxl_comm_result = false;

  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VALUE;
  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE;
  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 = -LIMIT_X_MAX_VELOCITY;
  }

  //dxl_comm_result = motor_driver.speedControl((int64_t)lin_vel1, (int64_t)lin_vel2);

  //Motor_Left(CW, (int64_t)lin_vel1); Motor_Right(CCW, (int64_t)lin_vel2);

  n20_Motor((int64_t)lin_vel1, (int64_t)lin_vel2);

  int64_left_msg.data =  lin_vel1;
  int64_right_msg.data =  lin_vel2;


  leftPwm_pub.publish(&int64_left_msg);
  rightPwm_pub.publish(&int64_right_msg);

  canDynamix_pub.publish(&str_msg);

}
