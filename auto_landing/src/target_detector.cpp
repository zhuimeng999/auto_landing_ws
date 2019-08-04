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
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  QRCodeDetector qrdetector;
public:
  MyDetector()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/uav0/usb_cam/image_raw", 1,
      &MyDetector::imageCb, this);
    image_pub_ = it_.advertise("/MyDetector/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~MyDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    Mat bbox;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if(qrdetector.detect(cv_ptr->image, bbox))
    {
      // Draw an example circle on the video stream
      int n = bbox.rows;
      for(int i = 0 ; i < n ; i++)
      {
        line(cv_ptr->image, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
      }

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MyDetector");
  MyDetector md;
  ros::spin();
  return 0;
}