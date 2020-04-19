#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multi_blob.hpp"
#include <cmath>
#include <list>


class blob{
// blob class that stores blob centroid/radius information    
    public:
        blob(int cent_j, int cent_i){  // instantiate class with centroid/radius
            i = cent_i;
            j = cent_j;
            rad = 1;
            N = 1;
            icm = i;
            jcm = j;
        }

        void check_blob(int, int);        
        int * return_params();  // returns pointers to params

    private:
        void update(int, int);

        int i;  // sum of i points
        int j;  // sum of j points
        int N;  // number of points in blob

        int icm;  // i centroid
        int jcm;  // j centroid
        int rad;  // radius

        int threshold = 100;
};

void blob::update(int jp, int ip){
// update blob centroid based on points new i, j value
// consider all points to have mass = 1 (http://hyperphysics.phy-astr.gsu.edu/hbase/cm.html)
    N += 1;
    i += ip;
    j += jp;
    
    icm = i/N;
    jcm = j/N; 
    rad = sqrt(N/3.1415);  // N can be interpreted as area of circle
}

void blob::check_blob(int jp, int ip){
    int dist = sqrt(pow(j-jcm, 2.0)+pow(i-icm, 2.0));
    if(dist < threshold){
        update(jp, ip);
    }
}


void multi_blob_track(cv_bridge::CvImagePtr&  cv_ptr){
    cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255, 0, 0));

    // initialize empty list to store blob objects
    std::list<blob*> blobs = {};

    // loop through pixels
    for(int j=0; j<cv_ptr->image.rows; j++){
        for(int i=0; i<cv_ptr->image.cols; i++){
            cv::Vec3b & intensity = cv_ptr->image.at<cv::Vec3b>(j,i);
            
            // check if pixel satisfies condition
            if(intensity.val[2] > 200 && intensity.val[0] < 150 && intensity.val[1] < 150){
                
            // assign pixels to blob
                if(blobs.size() == 0){
                    blob* temp_blob = new blob (j,i);
                    blobs.push_back(temp_blob);  // new is c++ equiv of calloc
                }else{
                    typedef std::list<blob*>::iterator blobsit;
                    for(blobsit k = blobs.begin(); k !=  blobs.end(); ++k){
                        (*k)->check_blob(j, i);
                    }
                } 
            }
        }
    }
}

