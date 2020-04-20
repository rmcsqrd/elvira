#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multi_blob.hpp"
#include <cmath>
#include <list>
#include <string>
#include <iostream>
#include <ros/console.h>
#include <algorithm>

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

        int check_blob(int, int);        
        void draw_circle(cv_bridge::CvImagePtr&);
        void update(int, int);

    private:
        int i;  // sum of i points
        int j;  // sum of j points
        int N;  // number of points in blob

        int icm;  // i centroid
        int jcm;  // j centroid
        int rad;  // radius

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

int blob::check_blob(int jp, int ip){
    int dist = sqrt(pow(j-jcm, 2.0)+pow(i-icm, 2.0));
    return dist;
}

void blob::draw_circle(cv_bridge::CvImagePtr& cv_ptr){
    cv::circle(cv_ptr->image, cv::Point(icm,jcm), 10, CV_RGB(0, 255, 0));
}


void multi_blob_track(cv_bridge::CvImagePtr&  cv_ptr){

// enable debugging messages
if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) { // Change the level to fit your needs
   ros::console::notifyLoggerLevelsChanged();
}
    // initialize empty list to store blob objects
    std::list<blob*> blobs = {};
    typedef std::list<blob*>::iterator blobsit;

    // loop through pixels
    for(int j=0; j<cv_ptr->image.rows; j++){
        for(int i=0; i<cv_ptr->image.cols; i++){
            cv::Vec3b & intensity = cv_ptr->image.at<cv::Vec3b>(j,i);
            
            // check if pixel satisfies condition
            //if(intensity.val[2] > 200 && intensity.val[0] < 150 && intensity.val[1] < 150){
            if(intensity.val[2] > 200){
                intensity.val[1] = 255;  // turn yellow to visually debug
            // assign pixels to blob
                if(blobs.size() == 0){
                    blob* temp_blob = new blob (j,i);
                    blobs.push_back(temp_blob);  // new is c++ equiv of calloc
                }else{
                    // loop through blob list and compute minimum blob dist
                    int blobdist [blobs.size()];
                    blobsit blob_id [blobs.size()];

                    int cnt = 0;
                    for(blobsit k = blobs.begin(); k !=  blobs.end(); ++k){
                        int dist = (*k)->check_blob(j, i);
                        blobdist[cnt] = dist;
                        blob_id[cnt] = k;
                        cnt += 1;
                    }
                    
                    // loop through blobdist array to find min because I need index
                    int mindex=0;
                    int threshold = 100;
                    for(int n = 0; n<blobs.size(); n++){
                        if(blobdist[n] < blobdist[mindex]){
                            mindex = n;
                        }
                    }
                    if(blobdist[mindex]<threshold){
                        (*blob_id[mindex])->update(j,i);
                    }else{
                        blobs.push_back(new blob (j,i));
                    }
                } 
            }
        }
    } 
    //ROS_DEBUG_STREAM(blobs.size());

    for(blobsit k = blobs.begin(); k != blobs.end(); ++k){
        (*k)->draw_circle(cv_ptr);
    }
}


