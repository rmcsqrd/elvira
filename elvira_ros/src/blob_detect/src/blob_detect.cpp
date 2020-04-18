#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/image_raw", 1,  // this is original
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, // this is for realsense
      &ImageConverter::imageCb, this);  // reference https://docs.ros.org/api/image_transport/html/classimage__transport_1_1ImageTransport.html#a1c847a2c719c874f84a78a6a60b98c7f
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 200 && cv_ptr->image.cols > 200){
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0,255,0));
    }

    // cycle through pixles and turn anything above a certain value of red into white
    for(int j=0; j<cv_ptr->image.rows; j++){
	    for(int i=0; i<cv_ptr->image.cols; i++){
            cv::Vec3b & intensity = cv_ptr->image.at<cv::Vec3b>(j, i);  // '&' passes by reference
            if(intensity.val[2] > 200){
                intensity.val[0] = 0;  // B
                intensity.val[1] = 255;  // G
                intensity.val[2] = 0;  // R
            }
	    }
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}