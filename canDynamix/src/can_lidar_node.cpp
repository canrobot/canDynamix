
#include "can_dynamix/canDynamix_lidar.h"


using namespace std;

canDynamix_lidar::canDynamix_lidar()
  : nh_priv_("~")
{
  // initialize tunnel node
  ROS_INFO("can_dynamix lidar Node Init");
  ROS_ASSERT(init());
}

canDynamix_lidar::~canDynamix_lidar()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool canDynamix_lidar::init()
{
	/*
  turning_radius_ = 0.08;
  rotate_angle_ = 50 * DEG2RAD;
  front_distance_limit_ = 0.4;
  side_distance_limit_  = 0.3;

  ROS_INFO("robot_model : turtlebot3_burger");
  ROS_INFO("turning_radius_ : %lf", turning_radius_);
  ROS_INFO("front_distance_limit_ = %lf", front_distance_limit_);
  ROS_INFO("side_distance_limit_ = %lf", side_distance_limit_);

  // initialize variables
  right_joint_encoder_ = 0.0;
  priv_right_joint_encoder_ = 0.0;
  robot_position_y = 0.0;
  robot_position_x = 0.0;

  imu_w = 0.0;
  imu_x = 0.0;
  imu_y = 0.0;
  imu_z = 0.0;

  cv_grid = cv::Mat::zeros(80,80,CV_8UC1);
  robot_pos_in_grid = cv::Mat::zeros(80,80,CV_8UC1);
  print_count = 0;
  first_top_count = 0;
  find_count = 0;

  top_data_x = 0;
  top_data_y = 0;

  position_flag = false;

  // initialize grid

  for(int grid_row = 0 ; grid_row < 80 ; grid_row ++)
  {
    for(int grid_col = 0 ; grid_col < 80 ; grid_col ++)
    {
      grid[grid_col][grid_row] = 0;
      grid_copy[grid_col][grid_row] = 0;
      closed_nodes_[grid_col][grid_row] = 0;
      open_nodes_[grid_col][grid_row] = 0;
    }
  }
  */

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<can_dynamix::cmdVelMsg>("/can_dynamix/lidar", 10);
  
  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &canDynamix_lidar::laserScanMsgCallBack,this);
  odom_sub_        = nh_.subscribe("/odom",10,&canDynamix_lidar::odomMsgCallBack,this);
  imu_sub_         = nh_.subscribe("/imu",10,&canDynamix_lidar::imuMsgCallBack,this);

  //exit_tunnel_pub_ = nh_.advertise<robit_master::driving_msg>("/tunnel_state", 1);
  //driving_mode_sub = nh_.subscribe("/driving_mode", 1, &Turtlebot3_Tunnel::drivingModeMsgCallBack, this);

  return true;
}

/*******************************************************************************
* Callback function
*******************************************************************************/
void canDynamix_lidar::imuMsgCallBack(const sensor_msgs::Imu::ConstPtr &imu)
{
    imu_w = imu->orientation.w;
    imu_z = imu->orientation.z;
    imu_x = imu->orientation.x;
    imu_y = imu->orientation.y;

    double yaw = atan2(2*imu_x*imu_y + 2*imu_w*imu_z, imu_w*imu_w + imu_x*imu_x - imu_y*imu_y - imu_z*imu_z);

    // get yaw angle of the turtlebot
    double yaw_angle = yaw * RAD2DEG;

    if(yaw_angle > -90.0 && yaw_angle <= 180.0)
      direction_angle = yaw_angle + 90.0;
    else if(yaw_angle > -180.0 && yaw_angle <= -90.0)
      direction_angle = yaw_angle + 450.0;
}

void canDynamix_lidar::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(position_flag == false)
  {
    robot_position_y_init = odom->pose.pose.position.x;
    robot_position_x_init = odom->pose.pose.position.y;

    position_flag = true;
  }

  else if(position_flag == true)
  {
    robot_position_y = odom->pose.pose.position.x - robot_position_y_init + 0.09;
    robot_position_x = odom->pose.pose.position.y - robot_position_x_init + 0.05;
  }
}

void canDynamix_lidar::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  {
		

    for (int scan_angle = 0; scan_angle < 360; scan_angle++)
    {
      if (std::isinf(scan->ranges.at(scan_angle))) // when the scan value is infinity
      {
        distance_obstacle[scan_angle] = scan->range_max; // range_max = 3.5
        is_obstacle = false;
      }
      else if(scan->ranges.at(scan_angle) == 0.0)
      {
        distance_obstacle[scan_angle] = 3.0;
        is_obstacle = false;
      }
      else
      {
        if(scan->ranges.at(scan_angle) < 0.5 && scan->intensities.at(scan_angle) > 3000)
        {
          distance_obstacle[scan_angle] = scan->ranges.at(scan_angle);
          is_obstacle = true;
        }
        else
          is_obstacle = false;
      }

	  //if(is_obstacle == false) ROS_INFO("laserScanMsgCallBack");
      

      if(is_obstacle == true) 
      {
		  
        //position_obstacle[scan_angle][POS_X] = (( robot_position_x) + (distance_obstacle[scan_angle] * cos((scan_angle + direction_angle) * DEG2RAD)));
        //position_obstacle[scan_angle][POS_Y] = (robot_position_y + (distance_obstacle[scan_angle] * sin((scan_angle + direction_angle) * DEG2RAD)));

		position_obstacle[scan_angle][POS_X] = (( robot_position_x) + (distance_obstacle[scan_angle] * cos((scan_angle) * DEG2RAD)));
        position_obstacle[scan_angle][POS_Y] = (robot_position_y + (distance_obstacle[scan_angle] * sin((scan_angle) * DEG2RAD)));

        grid_pos_x = position_obstacle[scan_angle][POS_X] * 40;
        grid_pos_y = position_obstacle[scan_angle][POS_Y] * 40;

        grid_pos_robot_x = robot_position_x * 40;
        grid_pos_robot_y = robot_position_y * 40;

        if(grid_pos_x >= 0 && grid_pos_y >= 0 && grid_pos_x < 80 && grid_pos_y < 80)
        {
         // cv_grid.at<uchar>(grid_pos_y , grid_pos_x) = OBSTACLE;
         // closed_nodes_[grid_pos_x][grid_pos_y] = OBSTACLE;
        }

        if(grid_pos_robot_x >= 0 && grid_pos_robot_y >= 0 && grid_pos_robot_x < 80 && grid_pos_robot_y < 80)
        {
          //robot_pos_in_grid.at<uchar>(grid_pos_robot_y , grid_pos_robot_x) = POS_ROBOT;
        }
		

		

		updatecommandVelocity(0.0, 0.0);


      }

	  ROS_INFO("grid_pos_x [%d], grid_pos_y [%d], direction_angle[%d] ", grid_pos_x, grid_pos_y, int(direction_angle)); 

	/*
      cv_grid_clone  = cv_grid.clone();

      cv::Mat mask_erode     = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
      cv::Mat mask_dilate    = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

      erode(cv_grid,
      cv_grid_clone,
      mask_erode,
      cv::Point(-1, -1),
      1);

      dilate(cv_grid,
      cv_grid_clone,
      mask_dilate,
      cv::Point(-1, -1),
      4);

      cv::add(cv_grid_clone, robot_pos_in_grid, cv_grid_clone);

      if(!(XFIN == grid_pos_robot_x && YFIN == grid_pos_robot_y))
      {
        pathFinding();
        find_path = false;
        find_direction(find_path);
      }

      else
      {
        find_path = true;
        find_direction(find_path);
      }
	  */
  }
}
}

/* update velocity */
void canDynamix_lidar::updatecommandVelocity(double linear, double angular)
{
  
    can_dynamix::cmdVelMsg cmd_vel;

    cmd_vel.linear  = linear;
    cmd_vel.angular = angular;
	cmd_vel.flag    = "lidar";

    cmd_vel_pub_.publish(cmd_vel);
  
}



int main(int argc, char **argv) 
{
	ros::init(argc, argv, "can_driving_node");

	canDynamix_lidar can_dynamix_lidar;
	can_dynamix_lidar.init();
	ros::Rate loop_rate(200);
	
  
	 while (ros::ok())
	  { 
		  ros::spinOnce();
          loop_rate.sleep();
		 // ros::spin();

	  }

	return 0;
}


