
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <dynamic_reconfigure/server.h>


namespace can_dynamix {

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  bool debug_view_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,  &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/can_dynamix/video_out", 1);

    ros::NodeHandle pnh_("~");
    pnh_.param("debug_view", debug_view_, false);
  }

  ~ImageConverter()
  {
//    if( debug_view_) {
//      cv::destroyWindow(OPENCV_WINDOW);
//    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat in_image, jout_image ;

        cv::Mat src_gray;	
	cv::Mat red_hue_image;
	cv::Mat lower_red_hue_range;
	cv::Mat upper_red_hue_range;


    try
    {
            in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  
        cv::cvtColor( in_image, src_gray, cv::COLOR_BGR2HSV );
	// Threshold the HSV image, keep only the red pixels // 0 - 10
	cv::inRange(src_gray, cv::Scalar(0, 150, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(src_gray, cv::Scalar(100, 100, 100), cv::Scalar(200, 255, 255), upper_red_hue_range);
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
	cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);     
      for( size_t i = 0; i < circles.size(); i++ )
      {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle( in_image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );   // center
        if (radius > 30)
        circle( in_image, center, radius, cv::Scalar(255,255,0), 5 );   // mark
      }
//        cv::cvtColor( lower_red_hue_range, jout_image, cv::COLOR_HSV2BGR );
        cv::cvtColor( red_hue_image, jout_image, cv::COLOR_GRAY2BGR );
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", in_image).toImageMsg();
      image_pub_.publish(out_img);

  }
};

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "can_image"); 



  while (ros::ok())
  { 
      can_dynamix::ImageConverter  imageCon ;
      ros::spin();

  }

  return 0;
}
