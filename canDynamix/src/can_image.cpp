
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
    image_sub_ = it_.subscribe("image", 1,  &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/can_dynamix/video_out", 1);

    ros::NodeHandle pnh_("~");
    pnh_.param("debug_view", debug_view_, false);
  }

  ~ImageConverter()
  {
    if( debug_view_) {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat in_image, jout_image ;
    try
    {
              cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  

      if (in_image.channels() > 1) {
        cv::cvtColor( in_image, jout_image, cv::COLOR_BGR2GRAY );
        /// Apply Canny edge detector
        cv::Canny( jout_image, jout_image, 50, 200, 3 );
      }
      else {
        /// Check whether input gray image is filtered such that canny, sobel ...etc
        bool is_filtered = true;
        for(int y=0; y < in_image.rows; ++y) {
          for(int x=0; x < in_image.cols; ++x) {
            if(!(in_image.at<unsigned char>(y, x) == 0
                 || in_image.at<unsigned char>(y, x) == 255)) {
              is_filtered = false;
              break;
            }
            if(!is_filtered) {
              break;
            }
          }
        }

        if(!is_filtered) {
          cv::Canny( jout_image, jout_image, 50, 200, 3 );
        }
      }


    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

      cv::cvtColor(jout_image, jout_image, CV_GRAY2BGR);

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 110 && cv_ptr->image.cols > 110) 
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 100, CV_RGB(255,0,0));

      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", jout_image).toImageMsg();

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
