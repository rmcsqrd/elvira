// this code shamelessly ripped from http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multi_blob.hpp"

void color_change(cv_bridge::CvImagePtr&  cv_ptr){
    // cycle through pixles and turn anything above a certain value of red into green
    for(int j=0; j<cv_ptr->image.rows; j++){
    	for(int i=0; i<cv_ptr->image.cols; i++){
         cv::Vec3b & intensity = cv_ptr->image.at<cv::Vec3b>(j, i);  // '&' passes by reference as a pro-gamer move
         if(intensity.val[2] > 200 && intensity.val[0] < 150 && intensity.val[1] < 150){
             intensity.val[0] = 0;  // B
             intensity.val[1] = 255;  // G
             intensity.val[2] = 0;  // R
            }
        }
    }
}

