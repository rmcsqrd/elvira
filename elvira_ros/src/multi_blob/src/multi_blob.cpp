#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multi_blob.hpp"

void multi_blob_track(cv_bridge::CvImagePtr&  cv_ptr){
    cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255, 0, 0));
}

class blob{
// blob class that stores blob centroid/radius information    
    public:
        blob(int cent_i, int cent_j){  // instantiate class with centroid/radius
            i = cent_i;
            j = cent_j;
            rad = 1;
            N = 1;
            icm = i;
            jcm = j;
        }

        void update_centroid(int, int);
        void update_radius(int);
        int * return_params();  // returns pointers to params

    private:
        int i;  // sum of i points
        int j;  // sum of j points
        int N;  // number of points in blob

        int icm;  // i centroid
        int jcm;  // j centroid
        int rad;  // radius
};

void blob::update_centroid(int ip, int jp){
// update blob centroid based on points new i, j value
// consider all points to have mass = 1 (http://hyperphysics.phy-astr.gsu.edu/hbase/cm.html)
    N += 1;
    i += ip;
    j += jp;
    
    icm = i/N;
    jcm = j/N; 
}

