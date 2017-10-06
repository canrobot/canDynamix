#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iostream>
#include "can_dynamix/cmdVelMsg.h"


using namespace cv;
using namespace std;


namespace can_dynamix {

//static const std::string OPENCV_WINDOW = "Image window";

class LaneDetect
{

  private:
	  //------------------------ Video subscriber & publisher ---------------------------------//
	  ros::NodeHandle nh;
	  image_transport::Subscriber sub_lanetrace;
	  image_transport::Publisher image_pub_;
	  //---------------------------------------------------------------------------------------//

	  //--------------------------- lanetrace publisher ------------------------------//
		ros::NodeHandle nh2;   // ROS NodeHandle
		ros::Publisher lanetrace_pub;  // ROS Topic Publishers
	  //------------------------------------------------------------------------------//

	  //--------------------------- lanetrace publisher ------------------------------//
		ros::NodeHandle nh3;			// ROS NodeHandle
		ros::Publisher pub_vel;          // ROS Topic Publishers
		can_dynamix::cmdVelMsg flagmsg;
	  //------------------------------------------------------------------------------//


    // int theta;    //test
    double theta_old;
    int median_num;
    double theta_save[5];
	
	 

	bool debug_view_;

  public:
	 

	long map(long x, long in_min, long in_max, long out_min, long out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
 
  
	LaneDetect() //: cmd(new geometry_msgs::Twist()), it_(nh_)
    {
	  //------------------------ image subscriber & publisher ---------------------//
	  image_transport::ImageTransport it(nh);
	  //sub_lanetrace = it.subscribe("/camera/image_lanetrace", 1, &LaneDetect::imageCallback,this);
	  sub_lanetrace = it.subscribe("/camera/rgb/image_raw", 1, &LaneDetect::imageCallBack,this);
	  image_pub_ = it.advertise("/can_dynamix/video_out", 1);
	  //---------------------------------------------------------------------------//

	

	  lanetrace_pub = nh2.advertise<can_dynamix::cmdVelMsg>("can_dynamix/lane", 1);

	  //pub_vel = nh3.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	  theta_old = 572;
	  median_num = 0;
    }

	~LaneDetect()
	{
		
	}
	
	
	void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
	{   // lane detection && lane control
	  try
	  {
			Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

			// if(frame.empty())
			//   break;

		//------------------------------------------- 차선 디텍션 ------------------------------------------//


		  // namedWindow( "result", WINDOW_AUTOSIZE);
		  // namedWindow( "lanetrace", WINDOW_AUTOSIZE);

		  //크기 설정 320*240
		  resize(frame, frame, Size(320, 240));
		  int width = frame.size().width;
		  int height = frame.size().height;

		  //--------------------------------------- 오른쪽 흰색 차선 ----------------------------------------------------//

		  Mat img_hsv, img_binary, img_edge;

		  //HSV 변환
		  cvtColor(frame, img_hsv, COLOR_BGR2HSV);
		  //흰색  H : 0,179  S : 0,16   V : 219,255
		  int LowH, LowS, LowV, HighH, HighS, HighV;
		  // LowH = 0;
		  // HighH = 179;
		  // LowS = 0;
		  // HighS = 16;
		  // LowV = 219;
		  // HighV = 255;
		  LowH = 0;
		  HighH = 179;
		  LowS = 0;
		  HighS = 16;
		  LowV = 202;
		  HighV = 255;

		  // 관심 HSV 영역을 제외한 나머지는 0으로
		  inRange(img_hsv, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), img_binary);

		  // morphological opening 작은 점들을 제거
		  erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		  dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		  //morphological closing 영역의 구멍 메우기
		  dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		  erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		  // Canny edge
		  Canny(img_binary, img_edge, 70, 210);

		  // 차선 관심영역 설정
		  vector<cv::Point> point_R;
		  point_R.push_back(Point(width/2+10,height));
		  point_R.push_back(Point(width/2+10, height-80));
		  point_R.push_back(Point(width, height-80));
		  point_R.push_back(Point(width,height));

		  Mat Mask = Mat::zeros(img_edge.size(), CV_8UC1);
		  fillConvexPoly(Mask, point_R, Scalar(255));   // 사다리꼴 마스크 만들기
		  Mat ROI_R;
		  bitwise_and(Mask, img_edge, ROI_R);

		  // HoughLinesP를 이용하여 직선 검출
		  vector<Vec4i> lines_R;
		  int threshold = 30;
		  HoughLinesP(ROI_R, lines_R, 1, CV_PI / 180, threshold, 10, 20);

		  // 기울기 20도부터 160도 사이의 선들만 검출
		  vector<cv::Point> lines_filter_R;  // 기울기로 필터된 라인
		  int x1, x2, y1, y2;
		  int slope_degree;
		  for (size_t i = 0; i < lines_R.size(); i++)
		  {
			x1= lines_R[i][0];
			y1= lines_R[i][1];
			x2= lines_R[i][2];
			y2= lines_R[i][3];
			slope_degree= (atan2((y1-y2),(x1-x2))*180)/CV_PI;

			if(abs(slope_degree) < 160 && abs(slope_degree) > 20)
			{

				lines_filter_R.push_back(Point(x1,y1));
				lines_filter_R.push_back(Point(x2,y2));

			}
		  }

		  Mat result4 = frame.clone();

		  // 검출된 선들중 대표 선 검출
		  Vec4f last_line_R;
		  double x_1, y_1, x_2, y_2, Ry1, Rx1, Ry2, Rx2;
		  if(!lines_filter_R.empty())
		  {
			fitLine(lines_filter_R,last_line_R, CV_DIST_L2, 0, 0.01, 0.01);
			x_1 = last_line_R[0];
			y_1 = last_line_R[1];
			x_2 = last_line_R[2];
			y_2 = last_line_R[3];
			Ry1 = height-10;
			Rx1 = (Ry1-y_2)/y_1*x_1 + x_2;
			Ry2 = height/2+40;
			Rx2 = (Ry2-y_2)/y_1*x_1 + x_2;

			line(result4, Point(int(Rx1),int(Ry1)), Point(int(Rx2),int(Ry2)), Scalar(0, 255, 0), 5, CV_AA);
		  }

		  //--------------------------------------------------------------------------------------------------------//




		 //--------------------------------------- 왼쪽 노란 차선 ----------------------------------------------------//
			//설명은 위의 오른쪽 흰색 차선과 동일함
			//노란색 H : 17,59  S : 21,98  V : 224,255
			// LowH = 17;
			// HighH = 59;
			// LowS = 21;
			// HighS = 98;
			// LowV = 224;
			// HighV = 255;
			
			/*
			LowH = 17;
			HighH = 179;
			LowS = 53;
			HighS = 255;
			LowV = 200;
			HighV = 255;
			*/

			LowH = 0;
			HighH = 255;
			LowS = 0;
			HighS = 255;
			LowV = 0;
			HighV = 120; //100 더 영역이 넓음

			inRange(img_hsv, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), img_binary);


			erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
			dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

			dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
			erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


			Canny(img_binary, img_edge, 70, 200);

			vector<cv::Point> point_L;
			point_L.push_back(Point(width/2-10,height));
			point_L.push_back(Point(width/2-10, height-80));
			point_L.push_back(Point(0, height-80));
			point_L.push_back(Point(0,height));

			Mask = Mat::zeros(img_edge.size(), CV_8UC1);
			fillConvexPoly(Mask, point_L, Scalar(255));
			Mat ROI_L;
			bitwise_and(Mask, img_edge, ROI_L);

			vector<Vec4i> lines_L;
			HoughLinesP(ROI_L, lines_L, 1, CV_PI / 180, threshold, 10, 20);

			vector<cv::Point> lines_filter_L;

			for (size_t i = 0; i < lines_L.size(); i++)
			{
			  x1= lines_L[i][0];
			  y1= lines_L[i][1];
			  x2= lines_L[i][2];
			  y2= lines_L[i][3];
			  slope_degree= (atan2((y1-y2),(x1-x2))*180)/CV_PI;

			  if(abs(slope_degree) < 160 && abs(slope_degree) > 20)
			  {

				  lines_filter_L.push_back(Point(x1,y1));
				  lines_filter_L.push_back(Point(x2,y2));

			  }
			}

			Vec4f last_line_L;
			double x_1_p, y_1_p, x_2_p, y_2_p, Ly1, Lx1, Ly2, Lx2;
			if(!lines_filter_L.empty())
			{
			  fitLine(lines_filter_L,last_line_L, CV_DIST_L2, 0, 0.01, 0.01);
			  x_1_p = last_line_L[0];
			  y_1_p = last_line_L[1];
			  x_2_p = last_line_L[2];
			  y_2_p = last_line_L[3];
			  Ly1 = height-10;
			  Lx1 = (Ly1-y_2_p)/y_1_p*x_1_p + x_2_p;
			  Ly2 = height/2+40;
			  Lx2 = (Ly2-y_2_p)/y_1_p*x_1_p + x_2_p;

			  line(result4, Point(int(Lx1),int(Ly1)), Point(int(Lx2),int(Ly2)), Scalar(0, 255, 0), 5, CV_AA);
			}


			//--------------------------------------------------------------------------------------------------------//





			//------------------------- vanishing point && theta ---------------------------------------------------//
			// 소실점 및 차선 각도 구하기

			double theta; // 차선 각도

			if(!lines_filter_R.empty() && lines_filter_L.empty())
			{ // 왼쪽차선만 없는경우
			  theta = (Ry2 - Ry1)/(Rx2 - Rx1);

			  if (theta > 1.428 && theta < 572)   // 흰색차선만 검출되도 각도가 45도보다 크면 직선경로라고 인식
			  {   //theta_degree = atan(theta)*180/CV_PI
				// 노란색 55도 , 흰색 124도
				theta = 572;    // 90도
				
			  }
			}
			else if(lines_filter_R.empty() && !lines_filter_L.empty())
			{   // 오른쪽차선만 없는경우
			  theta = (Ly2 - Ly1)/(Lx2 - Lx1);
			  if (theta < -1.428 && theta > -572)   // 노란차선만 검출되도 각도가 135도보다 크면 직선경로라고 인식
			  {
				theta = 572;    // 90도
				
			  }
			}
			else if(!lines_filter_R.empty() && !lines_filter_L.empty())
			{   // 두차선 모두 있는 경우

			  // vanishing point 계산
			  double y_van = ((y_2*y_1_p*x_1 - y_2_p*y_1*x_1_p + y_1*y_1_p*(x_2_p-x_2)))/(y_1_p*x_1 - y_1*x_1_p);
			  double x_van = (y_van - y_2)/y_1*x_1 + x_2;


			  double theta1 = (Ry2 - Ry1)/(Rx2 - Rx1); // 오른쪽차선 기울기
			  double theta2 = (Ly2 - Ly1)/(Lx2 - Lx1); // 왼쪽차선 기울기

			  double x1 = x_van - (y_van - 240)/theta1;
			  double x2 = x_van - (y_van - 240)/theta2;

			  int x_mid = (x1+x2)/2;


			   theta = (240 - y_van)/(160 - x_mid);
			   

			  //  theta = 572;
			}
			else
			{ // 둘 다 없는 경우
			  // 전의 세타 값으로 그대로
			  theta = theta_old;        // 이값 수정해야될듯요 ㅜㅜ
			}

			double theta_degree = atan(theta)*180/CV_PI;
			if(theta < 0)
			{
			  theta_degree = theta_degree + 180;
			}
			//theta_degree = 180 - theta_degree;

			// cout << "theta!!!!! : " << theta << endl;
			// cout << "theta_degree!!!!! : " << theta_degree << endl;

			theta_old = theta;    // 기울기 저장

			// 기울기 각도 그리기
			int direction_x, direction_y;
			direction_y = 210;
			direction_x = (direction_y - 240)/theta + 160;  //원래 160


			line(result4, Point(160,240), Point(int(direction_x),int(direction_y)), Scalar(0, 0, 255), 5, CV_AA);


		    std::cout << "  direction: ("<< direction_x <<"," << direction_y <<") " << "  R1: ("<< Rx1 << ") " << "  L1: ("<< Lx1 <<")\n";


		  // imshow("result", frame);
		  //imshow("lanetrace", result4);
		  sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", result4).toImageMsg();
		  image_pub_.publish(out_img);


		// waitKey(30);
		//waitKey(10);




			////////////////////////////// publisher //////////////////////////////////////////////////////
			theta_save[median_num] = theta_degree;


			if(median_num == 4)   // theta 5개 합쳐서 그중에 중간값 theat를 publisher로.. 홀수만
			{
					int i=0, j=0;
					double temp=0;
			 for(j=0;j<(median_num+1);j++)
			 {
				  for(i=0;i<median_num;i++)
				  {
					   if(theta_save[i]>theta_save[i+1])
					   {
							temp=theta_save[i];
							theta_save[i]=theta_save[i+1];
							theta_save[i+1]=temp;
					   }
				  }
			 }
			 theta_degree = theta_save[median_num/2];


			  flagmsg.theta = theta_degree;
			  int minVel = 240, maxVel = 300;
			  int Rx1_old, Lx1_old;
			  

			  geometry_msgs::Twist vel_msg;

			  float control_angular_vel;
			  float control_linear_vel;
			 
              //if(theta_degree > 150) control_angular_vel = 0.3;
			  Rx1_old = Rx1;
			  Lx1_old = Lx1;

			  //if( minVel < Rx1 && Rx1 < maxVel)flagmsg.flag = 1; //전진
			  //if( maxVel < Rx1 ) flagmsg.flag = 2;				//왼쪽으로 돌아감 우회전 필요
			  //if( minVel > Rx1 ) flagmsg.flag = 3;				//오른족으로 돌아감 좌회전 필요

			  if( Rx1 > 300 && Lx1 < 0)       flagmsg.flag = 1; //전진
			  if( Lx1 > 0  && Rx1 > 280 && Lx1 != Lx1_old) flagmsg.flag = 2; //왼쪽으로 돌아감 우회전 필요
			  if( Rx1 < 280 && Lx1 < -20 && Rx1 != Rx1_old) flagmsg.flag = 3; //오른족으로 돌아감 좌회전 필요
				

			//  if (flagmsg.flag == 1) { control_linear_vel = 0.3; control_angular_vel = 0.0; }
			//  if (flagmsg.flag == 2) { control_linear_vel = 0.1; control_angular_vel = -2; }
			//  if (flagmsg.flag == 3) { control_linear_vel = 0.1; control_angular_vel = 2; }
			  
			//	  vel_msg.linear.x = control_linear_vel;
			//	  vel_msg.linear.y = 0;
			//	  vel_msg.linear.z = 0;
			//	  vel_msg.angular.x = 0;
			//	  vel_msg.angular.y = 0;
			//	  vel_msg.angular.z = control_angular_vel;
			  
			  lanetrace_pub.publish(flagmsg);
			  //pub_vel.publish(vel_msg);

			  median_num = 0;
			}

			median_num++;

			//ros::spinOnce();

	   }


	  catch (cv_bridge::Exception& e)
	  {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
	} // end of imageCallBack
  };  // end of class
}     // emd of namespace


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "can_driving_node");
  
	 while (ros::ok())
	  { 
		  can_dynamix::LaneDetect  imageCon;
		  ros::spin();

	  }

	return 0;
}
