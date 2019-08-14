#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "auto_landing/qrcode.hpp"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class MyDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;

  QRCodeDetector qrdetector;
public:
  MyDetector()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = nh_.subscribe("/uav0/usb_cam/image_raw", 1,
      &MyDetector::imageCb, this);
    image_pub_ = nh_.advertise<const sensor_msgs::Image>("/MyDetector/output_video", 1);
    namedWindow(OPENCV_WINDOW);
  }

  ~MyDetector()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    Mat bbox;
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImage oimg;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Canny(cv_ptr->image, oimg.image, 30, 90, 5);

    imshow(OPENCV_WINDOW, cv_ptr->image);
    image_pub_.publish(oimg.toImageMsg());
  }

  void spin(void)
  {
    ros::spin();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MyDetector");
  MyDetector md;
  md.spin();
  return 0;
}