/* Authors: kyungman shin */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "motor.h"


#define WHEEL_RADIUS                     0.0165          // meter
#define WHEEL_SEPARATION                 0.086           // meter (canDynamix n20 motor)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                     -2147483648      // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT  0
#define RIGHT 1

#define CONTROL_MOTOR_SPEED_PERIOD        30   //hz

#define VELOCITY_CONSTANT_VALUE           135.1090523 // V = r * w = 1400 / 2 * PI * r 
                                                      // = 0.0165 * 0.229 * Goal RPM * 0.10472
                                                      // Goal Speed = V * 1263.632956882

// Limit values (n20 encoder motor)
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
std_msgs::Int32 int32_left_msg;
ros::Publisher left_enc_cnt_pub("/canDynamix/left_encoder", &int32_left_msg);

std_msgs::Int32 int32_right_msg;
ros::Publisher right_enc_cnt_pub("/canDynamix/right_encoder", &int32_right_msg);



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

  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  
  pinMode(13, OUTPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();  
  nh.subscribe(cmd_vel_sub);
  nh.advertise(left_enc_cnt_pub);
  nh.advertise(right_enc_cnt_pub);
}

void loop() {
  
  if ((millis()-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    controlMotorSpeed();
    tTime[0] = millis();
  }

  
  if ((millis()-tTime[1]) >= (1000 / 100))
  {
    controlMotorSpeed();
    tTime[1] = millis();

    int32_left_msg.data  = motorGetCounter(L_MOTOR);
    int32_right_msg.data = motorGetCounter(R_MOTOR);

    left_enc_cnt_pub.publish(&int32_left_msg);
    right_enc_cnt_pub.publish(&int32_right_msg);      
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

  
  motorMoveSpeed((int32_t)lin_vel1, (int32_t)lin_vel2);
}