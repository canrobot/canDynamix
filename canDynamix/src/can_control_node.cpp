// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Kei Okada nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
// https://github.com/robotpilot
/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

/*
   can_dynamix team 
   JinYoung Kim

   2017-09-03
*/


#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <can_dynamix/FindredMsg.h>
//#include <can_dynamix_blockbar/cmdVelMsg.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"
#include "can_dynamix/cmdVelMsg.h"
#define QSIZE 50

//using namespace std; 
//namespace can_dynamix{

//#define test

	std::string cmsg, lmsg, rmsg, bmsg;
    	geometry_msgs::Twist cmd; 
        int mark_id; 

ros::Publisher msg_pub; 
ros::Publisher vel_pub; 
//	ros::Subscriber sub; 
ros::Subscriber lidarScan_sub;  
ros::Subscriber lane_sub;     
ros::Subscriber findred_sub;  
ros::Subscriber mark_sub;     
//ros::Subscriber blockbar_sub; 
ros::Subscriber remocon_sub;  

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void getmsg_Callback(const std_msgs::String::ConstPtr& c_msg)
{
	ROS_INFO("catch message [%s] ", c_msg->data.c_str()); 

}




void lane_Callback(const can_dynamix::cmdVelMsg::ConstPtr& lane_msg)
{
	//ROS_INFO("lane detect message [%s] ", lane_msg->data.c_str()); 

	if( lmsg == "start" ){

	float theta = lane_msg->theta;
	std::string   flag  = lane_msg->flag;
    float control_linear_vel ;  
    float control_angular_vel ;
	
	/*
	ROS_INFO("lanetrace");
      	if(theta > 30 && flag == 1 ) { control_linear_vel = 0.1; control_angular_vel = 0; } //전진
		if(flag == 2 )               { control_linear_vel = 0.05; control_angular_vel = -0.3; } //우회전(왼쪽 선과 만남)
		//if(theta > 130 && flag == 2 ){ control_linear_vel = 0.05; control_angular_vel = -0; } //급 우회전 
        if( flag == 3 )              { control_linear_vel = 0.05; control_angular_vel = 0.3; } //좌회전(오른쪽 선과 만남)
		//if(theta < 30 && flag == 3 ) { control_linear_vel = 0.05; control_angular_vel = 1; } //급 좌회전 
		
		cmd.linear.x = control_linear_vel;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = control_angular_vel;
		*/

		if((theta-90) > 0 )
        {   // 좌회전

          float theta_diff = abs(theta-90);
          if(theta_diff < 5)
          {
			float control_linear_vel = 0.1;   // 직선 도로에서는 0.1 속도로 유지
            float control_angular_vel = 0;	

            cmd.linear.x = control_linear_vel;
            cmd.linear.y = 0;
            cmd.linear.z = 0;
            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = control_angular_vel;

          }
          else
          {

            float control_linear_vel = 0.05;    // 감속
            float control_angular_vel;
            //control_angular_vel = (0.19/65)*theta_diff + 0.0046;
			control_angular_vel = -0.3; //(0.19/65)*theta_diff + 0.0046;

            cmd.linear.x = control_linear_vel;
            cmd.linear.y = 0;
            cmd.linear.z = 0;
            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = control_angular_vel;
          }

        }
        else
        {   // 우회전

          float theta_diff = abs(theta-90);

          if(theta_diff < 5)
          {

            float control_linear_vel = 0.1;   // 직선 도로에서는 0.1 속도로 유지
            float control_angular_vel = 0;

            cmd.linear.x = control_linear_vel;
            cmd.linear.y = 0;
            cmd.linear.z = 0;
            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = control_angular_vel;

          }
          else
          {

            float control_linear_vel = 0.05;    // 감속
            float control_angular_vel;
            //control_angular_vel = -(0.19/65)*theta_diff - 0.0046;
			control_angular_vel = 0.2; //0(0.19/65)*theta_diff + 0.0046;

            cmd.linear.x = control_linear_vel;
            cmd.linear.y = 0;
            cmd.linear.z = 0;
            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = control_angular_vel;
          }

        }

	} //end of if( lmsg == "start") {}

}


void lidar_Callback(const can_dynamix::cmdVelMsg::ConstPtr& lidar_msg)
{
	//ROS_INFO("lidar range message [%s] ", lidar_msg->data.c_str());
	float theta  = lidar_msg->theta;
	std::string   flag  = lidar_msg->flag;
    float control_linear_vel = lidar_msg  -> linear ;  
    float control_angular_vel = lidar_msg -> angular ;

	cmd.linear.x = control_linear_vel;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = control_angular_vel;

}


void mark_Callback(const std_msgs::Float32MultiArray & mark_msg)
{

    if(mark_msg.data.size())
    {
		for(unsigned int i=0; i<mark_msg.data.size(); i+=12)
		{
			// get data
			mark_id = (int)mark_msg.data[i];
 #ifdef test
  	                ROS_INFO("mark detected message [%d] ", mark_id); 
  	                ROS_INFO("mark 0 [%d] ", (int)mark_msg.data[0]); 
  	                ROS_INFO("mark 1 [%f] ", mark_msg.data[1]); 
  	                ROS_INFO("mark 2 [%f] ", mark_msg.data[2]); 
  	                ROS_INFO("mark 3 [%f] ", mark_msg.data[3]); 
  	                ROS_INFO("mark 4 [%f] ", mark_msg.data[4]); 
 #endif
		}
    }
    else
    {
//    	printf("No objects detected.\n");
        mark_id  = 0;
    }
}


void findred_Callback(const can_dynamix::FindredMsg::ConstPtr& red_msg)
{
 #ifdef test
	ROS_INFO("findred message [%s] ", red_msg->sign_result.c_str()); 
        rmsg = red_msg->sign_result.c_str();
 #endif
}

/*
void blockbar_Callback(const can_dynamix_blockbar::cmdVelMsg::ConstPtr& bar_msg)
{
 #ifdef test
	ROS_INFO("blockbar message [%s] ", bar_msg->sign_result.c_str()); 
        bmsg = bar_msg->sign_result.c_str();
 #endif
}
*/
void remocon_Callback(const std_msgs::String::ConstPtr& comm_msg)
{
	cmsg = comm_msg->data.c_str();   // data내용을 문자열 타잎 으로 변환 후 cmsg에 넣는다. 
        if (cmsg == "start")  // 이를 비교할 "start"와 같으면 처리한다. 
	{ 
	   lmsg = "start";
	   //cmd.linear.x = 0.040;
	   ROS_INFO("Start driving");
	}
        else if (cmsg == "stop")
	{
  	   cmd.linear.x = 0.0;
	   ROS_INFO("Stop driving");

	}
 #ifdef test
	ROS_INFO("remocon cmd message [%s] ", comm_msg->data.c_str());   // c_str()문자열스트링 

 #endif
}

int main(int argc, char **argv)
{

	ros::init(argc,argv,"can_control");
	ros::NodeHandle n;
	msg_pub = n.advertise<std_msgs::String>("topic_message",QSIZE);
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",QSIZE);
    lidarScan_sub =n.subscribe("can_dynamix/lidar",QSIZE, lidar_Callback); 
	lane_sub      = n.subscribe("can_dynamix/lane", QSIZE,lane_Callback);
	findred_sub   = n.subscribe("can_dynamix/findred", QSIZE,findred_Callback);
	mark_sub      = n.subscribe("objects", QSIZE,mark_Callback);
	//blockbar_sub  = n.subscribe("can_dynamix/blockbar", QSIZE,blockbar_Callback);
	remocon_sub   = n.subscribe("can_dynamix/command", QSIZE,remocon_Callback);
	ros::Rate loop_rate(10);


  	cmd.linear.x = 0.0;
  	cmd.linear.y = 0.0;
  	cmd.linear.z = 0.0;
  	cmd.angular.x = 0.0;
  	cmd.angular.y = 0.0;
  	cmd.angular.z = 0.0;



	while(ros::ok())
	{
	   std_msgs::String msg;
	   std::stringstream ss;
	   ss<< "This Topic message from  Can_dynamix control ";
	   msg.data = ss.str();  
	   if((cmsg == "start" && (lmsg == "stop" || lmsg == "stop" || rmsg == "stop" || bmsg == "stop")) || cmsg == "stop")
		 cmd.linear.x = 0.0 ;
	   if (cmsg == "start" && (lmsg == "go" || lmsg == "go" || rmsg == "go" || bmsg == "go")) 
		 cmd.linear.x = 0.040; 

           if(mark_id == 1)	ROS_INFO("parking Mode ");
           if(mark_id == 2)	ROS_INFO("blockbar Mode "); 
           if(mark_id == 3)	ROS_INFO("turnel Mode ");            
           if(mark_id == 4)	ROS_INFO("slow Mode ");

//	   msg_pub.publish(msg);
	   vel_pub.publish(cmd);

	   ros::spinOnce();
	   loop_rate.sleep();
//	   ROS_INFO(" Sleep ");
	}

}
