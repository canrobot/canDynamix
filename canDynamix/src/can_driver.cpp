#include "ros/ros.h" 
#include "can_dynamix/cmdVelMsg.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "can_driver"); 
  ros::NodeHandle n;
  ros::Publisher pub_vel = n.advertise<can_dynamix::cmdVelMsg>("cmd_vel", 1000); 
  ros::Rate loop_rate(10);



  while (ros::ok())
  { 
    can_dynamix::cmdVelMsg vel_msg;

    //vel_msg.vel_linear = 3.0;
    //vel_msg.vel_angular = 0.0;

 

    //ROS_INFO("Message [vel_linear vel_angular] was published as: [%.3f %.3f]", vel_msg.vel_linear, vel_msg.vel_angular);


    //pub_vel.publish(vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} 
