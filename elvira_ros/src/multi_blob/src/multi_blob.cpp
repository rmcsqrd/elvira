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

        int check_blob(int*, int*);        
        void draw_circle(cv_bridge::CvImagePtr&);
        void update(int*, int*);

    private:
        int i;  // sum of i points
        int j;  // sum of j points
        int N;  // number of points in blob

        int icm;  // i centroid
        int jcm;  // j centroid
        int rad;  // radius

};

void blob::update(int *jp, int *ip){
// update blob centroid based on points new i, j value
// consider all points to have mass = 1 (http://hyperphysics.phy-astr.gsu.edu/hbase/cm.html)
    N += 1;
    i += *ip;
    j += *jp;
    
    icm = i/N;
    jcm = j/N; 
    rad = sqrt(N/3.1415);  // N can be interpreted as area of circle
}

int blob::check_blob(int* jp, int* ip){
    int dist = sqrt(pow(*jp-jcm, 2.0)+pow(*ip-icm, 2.0));
    return dist;
}

void blob::draw_circle(cv_bridge::CvImagePtr& cv_ptr){
    cv::circle(cv_ptr->image, cv::Point(icm,jcm), rad, CV_RGB(0, 255, 0));
}

void new_blob(std::list<blob*> *bloblist, int* j, int* i){
// helper function that adds blob to bloblist
    blob* new_blob = new blob (*j, *i);
    (*bloblist).push_back(new_blob);
}

void multi_blob_track(cv_bridge::CvImagePtr&  cv_ptr){

// define parameters within scope of this function
int threshold = 100;
int blue_max = 100;
int green_max = 100;
int red_max = 200;

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
            if(intensity.val[2] > red_max){
                //intensity.val[1] = 255;  // turn yellow to visually debug
            // assign pixels to blob
                if(blobs.size() == 0){
                    new_blob(&blobs, &j, &i);
                }else{
                    // iterate through blobs and compute distance to centroids
                    std::vector<int> blob_dist;
                    int dist_cnt = 0;
                    for(blobsit k = blobs.begin(); k !=  blobs.end(); ++k){
                        blob_dist.push_back((*k)->check_blob(&j, &i));
                        dist_cnt += 1;
                    }

                    // find minimum distance and compare to threshold
                    int dist_mindex = std::min_element(blob_dist.begin(), blob_dist.end())-blob_dist.begin();
                    int dist_min = *std::min_element(blob_dist.begin(), blob_dist.end());
                    if(dist_min > threshold){
                        new_blob(&blobs, &j, &i);
                    }else{
                        blobsit it = blobs.begin();
                        advance(it, dist_mindex);
                        (*it)->update(&j, &i);
                        
                    }
                } 
            }
        }
    } 
    ROS_DEBUG_STREAM("Frame Iteration Complete");
    //ROS_DEBUG_STREAM(blobs.size());

    for(blobsit k = blobs.begin(); k != blobs.end(); ++k){
        (*k)->draw_circle(cv_ptr);
    }
}

