
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

using namespace std;
using namespace cv;
clock_t start, stop;



namespace can_dynamix {

static const std::string OPENCV_WINDOW = "Image window";

class LaneDetect
{

  private:
	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	//geometry_msgs::TwistPtr cmd;
	//ros::Publisher vel_pub_   = nh_.advertise<geometry_msgs::Twist>("can_dynamix/cmd_vel", 100); 
	 

	bool debug_view_;

  public:
	Mat currFrame; //stores the upcoming frame
    Mat temp;      //stores intermediate results
    Mat temp2;     //stores the final lane segments
	Mat in_image, jout_image, line_image;
 
    int diff, diffL, diffR;
    int laneWidth;
    int diffThreshTop;
    int diffThreshLow;
    int ROIrows;
    int vertical_left;
    int vertical_right;
    int vertical_top;
    int smallLaneArea;
    int longLane;
    int  vanishingPt;
    float maxLaneWidth;
 
    //to store various blob properties
    Mat binary_image; //used for blob removal
    int minSize;
    int ratio;
    float  contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
    Size2f sz;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RotatedRect rotated_rect;
 
    Point pt1, pt2, pt3, pt4;
    float angle, angle2;
 
  
	LaneDetect() : it_(nh_)//: cmd(new geometry_msgs::Twist()), it_(nh_)
    {
		
		
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,  &LaneDetect::imageCallBack, this);
		image_pub_ = it_.advertise("/can_dynamix/video_out", 100);
		ros::NodeHandle pnh_("~");
		pnh_.param("debug_view", debug_view_, false);
    }

	~LaneDetect()
	{
		if( debug_view_) {
			cv::destroyWindow(OPENCV_WINDOW);
		}
	}
	
	void canDriving(Mat startFrame) 
	{
				
        currFrame = startFrame;                                    //if image has to be processed at original size
 
        currFrame = Mat(320,480,CV_8UC1,0.0);                        //initialised the image size to 320x480
        resize(startFrame, currFrame, currFrame.size());             // resize the input to required size
 
        temp      = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
        temp2     = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks
 
        vanishingPt    = currFrame.rows/2;                           //for simplicity right now
        ROIrows        = currFrame.rows - vanishingPt;               //rows in region of interest
        minSize        = 0.00015 * (currFrame.cols*currFrame.rows);  //min size of any region to be selected as lane
        maxLaneWidth   = 0.025 * currFrame.cols;                     //approximate max lane width based on image size
        smallLaneArea  = 7 * minSize;
        longLane       = 0.3 * currFrame.rows;
        ratio          = 4;
 
        
 
        //these mark the possible ROI for vertical lane segments and to filter vehicle glare
        vertical_left  = 2*currFrame.cols/5;
        vertical_right = 3*currFrame.cols/5;
        vertical_top   = 2*currFrame.rows/3;
 
        //namedWindow("lane",2);
        //namedWindow("midstep", 2);
        //namedWindow("currframe", 2);
        //namedWindow("laneBlobs",2);
 
        getLane();
		
    }

	void updateSensitivity()
    {
        int total=0, average =0;
        for(int i= vanishingPt; i<currFrame.rows; i++)
            for(int j= 0 ; j<currFrame.cols; j++)
                total += currFrame.at<uchar>(i,j);
        average = total/(ROIrows*currFrame.cols);
        cout<<"average : "<<average<<endl;
    }

	void getLane()
    {
        //medianBlur(currFrame, currFrame,5 );
        //updateSensitivity();
        //ROI = bottom half
        for(int i=vanishingPt; i<currFrame.rows; i++)
            for(int j=0; j<currFrame.cols; j++)
            {
                temp.at<uchar>(i,j)    = 0;
                temp2.at<uchar>(i,j)   = 0;
            }
 
        //imshow("currframe", currFrame);
        blobRemoval();
    }

	void markLane()
    {
        for(int i=vanishingPt; i<currFrame.rows; i++)
        {
            //IF COLOUR IMAGE IS GIVEN then additional check can be done
            // lane markings RGB values will be nearly same to each other(i.e without any hue)
 
            //min lane width is taken to be 5
            laneWidth =5+ maxLaneWidth*(i-vanishingPt)/ROIrows;
            for(int j=laneWidth; j<currFrame.cols- laneWidth; j++)
            {
 
                diffL = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j-laneWidth);
                diffR = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j+laneWidth);
                diff  =  diffL + diffR - abs(diffL-diffR);
 
                //1 right bit shifts to make it 0.5 times
                diffThreshLow = currFrame.at<uchar>(i,j)>>1;
                //diffThreshTop = 1.2*currFrame.at<uchar>(i,j);
 
                //both left and right differences can be made to contribute
                //at least by certain threshold (which is >0 right now)
                //total minimum Diff should be atleast more than 5 to avoid noise
                if (diffL>0 && diffR >0 && diff>5)
                    if(diff>=diffThreshLow /*&& diff<= diffThreshTop*/ )
                        temp.at<uchar>(i,j)=255;
            }
        }
 
    }
 
    void blobRemoval()
    {
        markLane();
 
        // find all contours in the binary image
        temp.copyTo(binary_image);
        findContours(binary_image, contours,
                     hierarchy, CV_RETR_CCOMP,
                     CV_CHAIN_APPROX_SIMPLE);
 
        // for removing invalid blobs
        if (!contours.empty())
        {
            for (size_t i=0; i<contours.size(); ++i)
            {
                //====conditions for removing contours====//
 
                contour_area = contourArea(contours[i]) ;
 
                //blob size should not be less than lower threshold
                if(contour_area > minSize)
                {
                    rotated_rect    = minAreaRect(contours[i]);
                    sz              = rotated_rect.size;
                    bounding_width  = sz.width;
                    bounding_length = sz.height;
 
 
                    //openCV selects length and width based on their orientation
                    //so angle needs to be adjusted accordingly
                    blob_angle_deg = rotated_rect.angle;
                    if (bounding_width < bounding_length)
                        blob_angle_deg = 90 + blob_angle_deg;
 
                    //if such big line has been detected then it has to be a (curved or a normal)lane
                    if(bounding_length>longLane || bounding_width >longLane)
                    {
                        drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);
                        drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
                    }
 
                    //angle of orientation of blob should not be near horizontal or vertical
                    //vertical blobs are allowed only near center-bottom region, where centre lane mark is present
                    //length:width >= ratio for valid line segments
                    //if area is very small then ratio limits are compensated
                    else if ((blob_angle_deg <-10 || blob_angle_deg >-10 ) &&
                             ((blob_angle_deg > -70 && blob_angle_deg < 70 ) ||
                              (rotated_rect.center.y > vertical_top &&
                               rotated_rect.center.x > vertical_left && rotated_rect.center.x < vertical_right)))
                    {
 
                        if ((bounding_length/bounding_width)>=ratio || (bounding_width/bounding_length)>=ratio
                                ||(contour_area< smallLaneArea &&  ((contour_area/(bounding_width*bounding_length)) > .75) &&
                                   ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)))
                        {
                            drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);
                            drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
                        }
                    }
                }
            }
        }
        //imshow("midstep", temp);
        //imshow("laneBlobs", temp2);
       // imshow("lane",currFrame);
 
    //--[ 라인검출하여 제어하기 ]------------------------------------------------------------------------------
        
        
        
        vector<Vec2f> lines;
        HoughLines(temp2, lines, 1, CV_PI / 180, 150, 0, 0);
        
         
        for (size_t i = 0; i < lines.size(); i++) // 검출된 포인트를 차선으로 연결.
        {
            float a1, a2;
            float rho = lines[i][0], theta = lines[i][1];
            float theta1, theta2, theta3;
            float rho1, rho2, rho3;
            int length = 800;
         
            if (theta<1.5 && theta>0)
            {
                theta1 = theta;
                rho1 = rho;
                double a = cos(theta1), b = sin(theta1);
                double x0 = a*rho1, y0 = b*rho1;
                 
                pt1.x = cvRound(x0 - length * (-b));
                pt1.y = cvRound(y0 - length * (a));
                pt2.x = cvRound(x0 + length * (-b));
                pt2.y = cvRound(y0 + length * (a));
                 
                angle = (atan2(pt1.y - pt2.y, pt1.x - pt2.x))*(180 / CV_PI);
                
             
            }
         
            else if (theta<3.14 && theta>2.0)
            {
                theta2 = theta;
                rho2 = rho;
                double a2 = cos(theta2), b2 = sin(theta2);
                double x02 = a2*rho2, y02 = b2*rho2;
                 
                pt3.x = cvRound(x02 - length * (-b2));
                pt3.y = cvRound(y02 - length * (a2));
                pt4.x = cvRound(x02 + length * (-b2));
                pt4.y = cvRound(y02 + length * (a2));
                angle2 = (atan2(pt3.y - pt4.y, pt3.x - pt4.x))*(180 / CV_PI);
                
             
            }
         
            if (angle > -60 && angle2 < 60)
            {
                //send_data(6);
            }
         
        } // Car Lane "for" End

		std::cout << " Left line: ("<< pt1 <<"," << pt2 << "," << angle << ") " << " Right line: ("<< pt3 <<"," << pt4 << "," << angle2 << ")\n";
		//std::cout << " Left line: ("<< pt1 <<"," << pt2 << ") " << " Right line: ("<< pt3 <<"," << pt4 << ")\n";
		 
 
    }

	void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		float angle;

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
		line_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
		
		try
		{
		
			//LaneDetect detect;
			canDriving(in_image);
			
		 
			//while(1)
			//{
			
			   

			   cvtColor(in_image, in_image, CV_BGR2GRAY);
		 
				//start = clock();
				nextFrame(in_image);
				//stop =clock();
				// cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;
		 
				
				cvtColor(currFrame, line_image , CV_GRAY2BGR); //currFrame와 line_image 동기화가 됨. 이상함. ^^
		 
				cv::Point txtPt(line_image.cols / 2 - 230, line_image.rows / 2 - 140);
				putText(line_image, "[CAN_Driving Test]", txtPt, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2, 8);
		 
				//---------------[화면에 선 그리기]-----------------------------


				if (pt1.x != 0 && pt3.x != 0) { //forward
					
				line(line_image, pt1, pt2, Scalar(255, 0, 0), 2, CV_AA);
				line(line_image, pt3, pt4, Scalar(0, 0, 255), 2, CV_AA);
		 
				}
				
		 
				else if (pt1.x != 0 && pt3.x == 0) { //left
				line(line_image, pt1, pt2, Scalar(255, 0, 0), 2, CV_AA);
		 
				
				}
				
				else if (pt1.x == 0 && pt3.x != 0) { //right
				line(line_image, pt3, pt4, Scalar(0, 0, 255), 2, CV_AA);
				
				}
				else {
				
				}

				//----------------------------------------------------------
				
					pt1.x = 0;            pt1.y = 0;
					pt2.x = 0;            pt2.y = 0;
					pt3.x = 0;            pt3.y = 0;
					pt4.x = 0;            pt4.y = 0;
					
				// Display the currFrame
				// namedWindow("Lane Detection", 2);
				// imshow("Lane Detection",line_image);

				sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", line_image).toImageMsg();
				image_pub_.publish(out_img);



				//vel_pub_.publish(cmd);

			//}
			
		}

		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}


	}
 
 
    void nextFrame(Mat &nxt)
    {
        //currFrame = nxt;                        //if processing is to be done at original size
 
        resize(nxt ,currFrame, currFrame.size()); //resizing the input image for faster processing
        getLane();
    }
 
    Mat getResult()
    {
        return temp2;
    }


  
 };

}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "can_image");
  
	 while (ros::ok())
	  { 
		  can_dynamix::LaneDetect  imageCon;
		  ros::spin();

	  }

	return 0;
}
