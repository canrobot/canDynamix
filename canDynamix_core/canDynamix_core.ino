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

#include "motor.h"
#include "MedianFilter.h"



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

#define DEG2RAD(x)                        (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                        (x * 57.2957795131)  // *180/PI
                                                      
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

sensor_msgs::Range sonic_msg;
ros::Publisher sonic_pub("/canDynamix/sonic_range", &sonic_msg);


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


/*******************************************************************************
* Declaration for sonic
*******************************************************************************/
MedianFilter sonic_filter(16, 0);
const int sonic_trigger_pin = A0; //Trig pin
const int sonic_echo_pin    = A1; //Echo pin
double    sonic_distance    = 0.0;


void setup() {

  motorBegin();
  Serial.begin(115200);

  motorMoveSpeed(0, 0);
  
  pinMode(13, OUTPUT);
  pinMode(sonic_trigger_pin, OUTPUT); // Trigger is an output pin
  pinMode(sonic_echo_pin, INPUT);     // Echo is an input pin

  nh.getHardware()->setBaud(115200);
  nh.initNode();  
  nh.subscribe(cmd_vel_sub);
  nh.advertise(left_enc_cnt_pub);
  nh.advertise(right_enc_cnt_pub);
  nh.advertise(sonic_pub);
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
    controlMotorSpeed();
    tTime[1] = millis();

    int32_left_msg.data  = motorGetCounter(L_MOTOR);
    int32_right_msg.data = motorGetCounter(R_MOTOR);

    left_enc_cnt_pub.publish(&int32_left_msg);
    right_enc_cnt_pub.publish(&int32_right_msg);      
  }
  
  
  // 50Hz
  if ((millis()-tTime[2]) >= (1000 / 50))
  {
    tTime[2] = millis();
   
    updateSonic();


    sonic_msg.field_of_view = DEG2RAD(10);  
    sonic_msg.max_range = 200.0 / 100.0;    //    2m
    sonic_msg.min_range =   5.0 / 100.0;    // 0.15m
    sonic_msg.header.frame_id = "base_scan";
    sonic_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

    sonic_msg.header.stamp = nh.now();
    sonic_msg.header.seq = seq_ctr++;
    sonic_msg.range = sonic_distance;    

    sonic_pub.publish(&sonic_msg);      
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
  int  duration;
  long distance_mm;

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

  distance_mm = (((double)duration / 2.9) / 2.0);     // Actual calculation in mm

  sonic_distance = (double)distance_mm / 1000.0;
}


