#ifndef CANDYNAMIX_CORE_NODE_H_
#define CANDYNAMIX_CORE_NODE_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <candynamix_msgs/sensor.h>



#define WHEEL_RADIUS                    (0.034/2)                           // 바퀴 반지름(meter)
#define WHEEL_SEPARATION                0.083                               // 바퀴간 거리(meter)

#define ENCODER_RES                     1400                                // 1바퀴 회전시 엔코더 펄스 수 
#define TICK2RAD                        (((360./ENCODER_RES) * M_PI) / 180.)  // 엔코더 1펄스일때 회전하는 각도(라디안) 


#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor

class CanDynamixCore
{
 public:
  CanDynamixCore();
  ~CanDynamixCore();
  bool init();
  bool update();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  // (TODO)

  // ROS Time
  ros::Time last_cmd_vel_time_;
  ros::Time prev_update_time_;
  ros::Time prev_time_;

  // ROS Topic Publishers
  ros::Publisher joint_states_pub_;
  ros::Publisher odom_pub_;

  sensor_msgs::JointState joint_states_;
  nav_msgs::Odometry odom_;
  tf::TransformBroadcaster tf_broadcaster_;

  sensor_msgs::Range sonic_;
  geometry_msgs::TransformStamped tfs_msg;
  ros::Publisher sonic_pub_;
  ros::Subscriber sensor_sub_;
  
  sensor_msgs::Imu imu_msg;
  ros::Publisher imu_pub_;
    

  double left_encoder;
  double right_encoder;

  double pre_left_encoder;
  double pre_right_encoder;

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;

  float  odom_pose_[3];
  float  odom_vel_[3];
  double pose_cov_[36];

  std::string joint_states_name_[2];

  double last_position_[2];
  double last_velocity_[2];

  double wheel_seperation_;
  double turning_radius_;
  double robot_radius_;

  // Function prototypes
  bool updateOdometry(ros::Duration diff_time);
  void updateJoint(void);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
  void updateIMU(const candynamix_msgs::sensor sensor_msg);

  void sensorCallback(const candynamix_msgs::sensor sensor_msg);



};

#endif // CANDYNAMIX_CORE_H_
