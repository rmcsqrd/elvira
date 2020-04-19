// this code shamelessly ripped from http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multi_blob.hpp"

void multi_blob_track(cv_bridge::CvImagePtr&  cv_ptr){
    
}

class blob{
    int cent_i, cent_j, rad;
    
    public:
        void update_centroid(int, int);
        void update_radius(int);
        int * return_params();  // returns pointers to params
}

void blob::update_centroid(cent_i, cent_j){

}

