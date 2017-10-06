/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*objectTrackingTutorial.cpp

Written by  Kyle Hounslow 2013

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*/
//https://www.youtube.com/watch?v=4KYlHgQQAts
//code : https://www.dropbox.com/s/o22cnih7v0mu7gv/multipleObjectTracking.cpp?dl=0

/*
 * can_dynamix Team
 * HakSeung Wang
 * 2017-09-03
 */

#include "ros/ros.h"

#include<stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include "std_msgs/String.h"
//#include "can_dynamix_blockbar/blockbar_msg.h"
#include "can_dynamix/Object.h"

#define test
//#define add_HoughLine

using namespace cv;
using namespace std;

#define QueueSize 19
ros::Publisher pub_sign;

image_transport::Publisher pub;
sensor_msgs::ImagePtr image_pub;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;//40*40
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;//1.5


int result_index = 0;
int last_y = 0;
int rows = 0;

void drawObject(vector<Object> Objects, Mat &frame){

	for(int i = 0; i < Objects.size(); i++){

	cv::circle(frame,cv::Point(Objects.at(i).getXPos(), Objects.at(i).getYPos()),10,cv::Scalar(0,0,255));
	cv::putText(frame,
                    format("(X:%d, Y:%d, num:%d)",Objects.at(i).getXPos(), Objects.at(i).getYPos(), i),
                    cv::Point(Objects.at(i).getXPos(), Objects.at(i).getYPos()+20),
                    1,
                    1,
                    Scalar(0,255,0));
	cv::putText(frame,
                    Objects.at(i).getType(),
                    cv::Point(Objects.at(i).getXPos(), Objects.at(i).getYPos()-30),
                    3,
                    1,
                    Objects.at(i).getColour());
        //ROS_INFO("%d번째, XPos : %d, YPos : %d", i, Objects.at(i).getXPos(), Objects.at(i).getYPos());
}
}

void detectObject(vector<Object> Objects){
    int y1 = 0, y2 = 0;

    for(int i = 0; i < Objects.size(); i++){

        y2 = last_y + 20;
        y1 = last_y - 20;

        if(y2 > rows) y2 = rows;
        else if(y1 < 0) y1 = 0;
        if(i == 1)
        {
            last_y = Objects.at(1).getYPos();
        if(Objects.at(0).getYPos() >= y1 && Objects.at(0).getYPos() <= y2){
            result_index = abs(Objects.at(0).getXPos()-Objects.at(1).getXPos());
        }}

    }

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

        Mat erodeElement = getStructuringElement(MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement(MORPH_RECT,Size(8,8));


        erode(thresh,thresh,erodeElement);
        erode(thresh,thresh,erodeElement);


        dilate(thresh,thresh,dilateElement);
        dilate(thresh,thresh,dilateElement);

}
void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed){
	
	vector<Object> Objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                                ROS_INFO("index : %d", index);
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object blockbar;				
					
					blockbar.setXPos(moment.m10/area);
					blockbar.setYPos(moment.m01/area);
					Objects.push_back(blockbar);


					objectFound = true;

				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(Objects,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}
void trackFilteredObject(Object attribute, Mat threshold,Mat HSV, Mat &cameraFeed){
	
	vector<Object> Objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;

        result_index = 0;

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
              //  ROS_INFO("Objects : %d", numObjects);
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
 //             ROS_INFO("index : %d", index);
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object blockbar;				
					
					blockbar.setXPos(moment.m10/area);
					blockbar.setYPos(moment.m01/area);
					blockbar.setType(attribute.getType());
					blockbar.setColour(attribute.getColour());
					
					Objects.push_back(blockbar);
                          //              ROS_INFO("XPos : %d, YPos : %d", blockbar.getXPos(), blockbar.getYPos());
					objectFound = true;

				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(Objects,cameraFeed);}
                                detectObject(Objects);
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    bool calibrationMode = false;//true, false
    Mat camera_image;
    Mat roi_image;
    Mat threshold;
    Mat HSV_image;

    try
    {
       camera_image = cv_bridge::toCvShare(msg, "bgr8")->image;

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    rows = camera_image.rows;

    if(calibrationMode==true){
        //imshow("Source", camera_image);
        //waitKey(1);

        //create window for trackbars
        //create trackbars and insert them into window
        //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
        //the max value the trackbar can move (eg. H_HIGH),
        //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
        //
        createTrackbar( " H_MIN :", "Source", &H_MIN, H_MAX, 0);
        createTrackbar( " H_MAX :", "Source", &H_MAX, H_MAX, 0);
        createTrackbar( " S_MIN :", "Source", &S_MIN, S_MAX, 0);
        createTrackbar( " S_MAX :", "Source", &S_MAX, S_MAX, 0);
        createTrackbar( " V_MIN :", "Source", &V_MIN, V_MAX, 0);
        createTrackbar( " V_MAX :", "Source", &V_MAX, V_MAX, 0);
    //if in calibration mode, we track objects based on the HSV slider values.
    cvtColor(camera_image,HSV_image,COLOR_BGR2HSV);
    inRange(HSV_image,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
    morphOps(threshold);
    //imshow("calibration", threshold);
    //waitKey(3);
    trackFilteredObject(threshold,HSV_image,camera_image);
    }else{
#ifdef test
        //imshow("Source", camera_image);
        //waitKey(1);
#endif
        roi_image = camera_image(Rect(0, 300, 640, 100));
#ifdef test
       // printf("입력영상의 크기 =%d x %d \n", camera_image.cols, camera_image.rows);
        //imshow("roi_image", roi_image);
        //waitKey(1);
#endif
    Object blockbar("Block Bar");//, traffic_light("traffic_light");
    cvtColor(roi_image,HSV_image,COLOR_BGR2HSV);
    inRange(HSV_image,blockbar.getHSVmin(),blockbar.getHSVmax(),threshold);
    morphOps(threshold);
    //imshow("Test", threshold);
    //waitKey(1);
#ifdef add_HoughLine
    Mat img_canny;
    Canny(threshold, img_canny, 70, 210);

    vector<Vec4i> lines;
    HoughLinesP(img_canny, lines, 1, CV_PI / 180, 30, 1, 5);

    Mat original = camera_image.clone();
    for( int i=0; i<lines.size(); i++ )
        {
            Vec4i L = lines[i];
            line(original, Point(L[0],L[1]), Point(L[2],L[3]),
                 Scalar(0,255,0), 3, LINE_AA );
            imshow("Blockbar_line", original);
            waitKey(1);
}
#endif
    trackFilteredObject(blockbar,threshold,HSV_image,roi_image);
}
#ifdef test
    //imshow("Result", camera_image);
    //waitKey(1);
	  image_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", camera_image).toImageMsg();
      pub.publish(image_pub);
#endif

    ROS_INFO("result : %d", result_index);

    std_msgs::String sign_msg;
	//std::stringstream ss;

    if(result_index >= 90 && result_index <=130)
      { sign_msg.data ="stop"; }
    else
    {
        sign_msg.data="go";
    }

          pub_sign.publish(sign_msg);

        waitKey(1);
}

int main(int argc, char **argv) 
{

  ros::init(argc, argv, "can_dynamix_blockbar_MCU");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  image_sub = it.subscribe("cv_camera/image_raw", QueueSize, imageCallback);

  pub = it.advertise("can_blockbar/image_raw", 1);

  pub_sign = nh.advertise<std_msgs::String>("can_blockbar", QueueSize);

  ros::spin();


  return 0;
}
