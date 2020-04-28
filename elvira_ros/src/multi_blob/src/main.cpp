// this code shamelessly ripped from http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multi_blob.hpp"
#include "std_msgs/Int32MultiArray.h"
static const std::string OPENCV_WINDOW = "Elvira Vision";
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;  // this subscribes from realsense
  image_transport::Publisher image_pub_;   // this publishes the image 
  ros::Publisher image_state_pub;          // this publishes to the Julia node
  ros::Subscriber julia_overlay_sub;          // this publishes to the Julia node
  ros::NodeHandle n;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/image_raw", 1,  // this is original
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, // this is for realsense
      &ImageConverter::imageCb, this);  // reference https://docs.ros.org/api/image_transport/html/classimage__transport_1_1ImageTransport.html#a1c847a2c719c874f84a78a6a60b98c7f
    image_pub_ = it_.advertise("/multi_blob/blob_overlay", 1);

    image_state_pub = n.advertise<std_msgs::Int32MultiArray>("/multi_blob/blob_data", 100, true);
    julia_overlay_sub = n.subscribe<std_msgs::Int32MultiArray>("/julia_brain/visual_out", 100, arrayCallback);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr multiblob_cv_ptr;
    cv_bridge::CvImagePtr julia_cv_ptr;
    try
    {
      multiblob_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // this points to the image that will be modified by the multiblob node
      julia_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);      // this points to the image that we overlay with output from the julia brain
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // detect blobs from discs
    multi_blob_track(multiblob_cv_ptr, &image_state_pub);

    // overlay output from julia node onto display image

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, multiblob_cv_ptr->image);  // uncomment this to see raw image feed from multiblob
    cv::imshow(OPENCV_WINDOW, julia_cv_ptr->image);  // uncomment this to see raw image feed from multiblob
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(multiblob_cv_ptr->toImageMsg());
  }
};

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array){
/*  PUT VISUAL OUTPUT CODE HERE
    int cnt = 0;
    int Arr[5];
    for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        if(i == 5){
            
        Arr[i] = *it;
        i++;
    }
*/
    return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
