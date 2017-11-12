/* Authors: Baram */

#include <candynamix_core/candynamix_core_node.h>




CanDynamixCore::CanDynamixCore()
: nh_priv_("~")
{
  //Init fake turtlebot node
  ROS_ASSERT(init());
}

CanDynamixCore::~CanDynamixCore()
{
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool CanDynamixCore::init()
{
  // initialize ROS parameter
		
	wheel_seperation_ = 0.085;
  turning_radius_   = 0.080;
  robot_radius_     = 0.105;

  nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  // initialize variables
  wheel_speed_cmd_[LEFT]  = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_   = 0.0;
  goal_angular_velocity_  = 0.0;
  cmd_vel_timeout_        = 1.0;
  last_position_[LEFT]    = 0.0;
  last_position_[RIGHT]   = 0.0;
  last_velocity_[LEFT]    = 0.0;
  last_velocity_[RIGHT]   = 0.0;

  left_encoder  = 0.0;
  right_encoder = 0.0;
  pre_left_encoder = 0.0;
  pre_right_encoder = 0.0;

  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2,0.0);
  joint_states_.velocity.resize(2,0.0);
  joint_states_.effort.resize(2,0.0);

  // initialize publishers
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);    
  sonic_pub_        = nh_.advertise<sensor_msgs::Range>("/canDynamix/sonic_range", 100);
  imu_pub_          = nh_.advertise<sensor_msgs::Imu>("imu", 100);
  

  sensor_sub_       = nh_.subscribe("/canDynamix/sensor", 100, &CanDynamixCore::sensorCallback, this);  
  

  prev_update_time_ = ros::Time::now();
  prev_time_ = ros::Time::now();


  return true;
}

void CanDynamixCore::sensorCallback(const candynamix_msgs::sensor sensor_msg)
{
  static bool started = false;
  static int32_t seq_ctr = 0;


  left_encoder  = sensor_msg.left_count;
  right_encoder = sensor_msg.right_count;

  if (started == false)
  {
    pre_left_encoder  = left_encoder;
    pre_right_encoder = right_encoder;
    started = true;
  }

  sonic_.field_of_view = DEG2RAD(10);  
  sonic_.max_range = 200.0 / 100.0;    //    2m
  sonic_.min_range =   2.0 / 100.0;    // 0.02m
  sonic_.header.frame_id = "base_scan";
  sonic_.radiation_type = sensor_msgs::Range::ULTRASOUND;

  sonic_.header.stamp = ros::Time::now();
  sonic_.header.seq = seq_ctr++;
  sonic_.range = ((sensor_msg.sonic_distance_time / 2.9) / 2) / 1000.0;    
  

  sonic_pub_.publish(sonic_);

  updateIMU(sensor_msg);
}


/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool CanDynamixCore::updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];


  wheel_speed_cmd_[LEFT]  = left_encoder  - pre_left_encoder;
  wheel_speed_cmd_[RIGHT] = right_encoder - pre_right_encoder;


  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = wheel_speed_cmd_[LEFT];
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = TICK2RAD * wheel_speed_cmd_[LEFT];
  wheel_r = TICK2RAD * wheel_speed_cmd_[RIGHT];

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);  
  
  
  // We should update the twist of the odometry
  odom_.twist.twist.linear.x  = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  pre_left_encoder = left_encoder;
  pre_right_encoder = right_encoder;

  return true;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void CanDynamixCore::updateJoint(void)
{
  joint_states_.position[LEFT]  = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void CanDynamixCore::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}
geometry_msgs::TransformStamped odom_tf;
/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void CanDynamixCore::updateIMU(const candynamix_msgs::sensor sensor_msg)
{
  imu_msg.header.stamp    = ros::Time::now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = sensor_msg.gx;
  imu_msg.angular_velocity.y = sensor_msg.gy;
  imu_msg.angular_velocity.z = sensor_msg.gz;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = sensor_msg.ax;
  imu_msg.linear_acceleration.y = sensor_msg.ay;
  imu_msg.linear_acceleration.z = sensor_msg.az;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = sensor_msg.qw;
  imu_msg.orientation.x = sensor_msg.qx;
  imu_msg.orientation.y = sensor_msg.qy;
  imu_msg.orientation.z = sensor_msg.qz;

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub_.publish(imu_msg);
  

  tfs_msg.header.stamp    = ros::Time::now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = sensor_msg.qw;  
  tfs_msg.transform.rotation.x = sensor_msg.qx;
  tfs_msg.transform.rotation.y = sensor_msg.qy;
  tfs_msg.transform.rotation.z = sensor_msg.qz;


  tfs_msg.transform.translation.x = 0.0;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;

  tf_broadcaster_.sendTransform(tfs_msg);
}


/*******************************************************************************
* Update function
*******************************************************************************/
bool CanDynamixCore::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  /*
  // zero-ing after timeout
  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }
  */

  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // joint_states
  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);
  
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "candynamix_core_node");
  CanDynamixCore cdCore;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    cdCore.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
