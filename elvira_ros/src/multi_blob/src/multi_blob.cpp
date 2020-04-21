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
#include <string>

class blob{
// blob class that stores blob centroid/radius information    
    public:
        blob(int cent_j, int cent_i, int* blob_color){  // instantiate class with centroid/radius
            blob_blue = *(blob_color);
            blob_green = *(blob_color+1);
            blob_red = *(blob_color+2);
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
        
        int blob_blue; 
        int blob_green;
        int blob_red; 

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
    cv::circle(cv_ptr->image, cv::Point(icm,jcm), rad, CV_RGB(blob_red, blob_green, blob_blue), -1);
    cv::drawMarker(cv_ptr->image, cv::Point(icm, jcm), CV_RGB(0, 0, 0), 0);
    
    // draw labels per per https://answers.opencv.org/question/27695/puttext-with-black-background/
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;
    std::string label = "I am a bubble";

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::rectangle(cv_ptr->image, cv::Point(icm-rad, jcm+rad) + cv::Point(0, baseline), cv::Point(icm-rad, jcm+rad) + cv::Point(text.width, -text.height), CV_RGB(0,0,0), cv::FILLED);
    cv::putText(cv_ptr->image, label, cv::Point(icm-rad, jcm+rad), fontface, scale, CV_RGB(255,255,255), thickness, 8);
}

int point_check(cv_bridge::CvImagePtr& cv_ptr, int*j, int*i){
// helper function that consolidates point to blob assignment conditional check
    int threshold = 200;

    int blue_max = 128;
    int green_max = 128;
    int red_max = 255;

    int blue_min = 0;
    int green_min = 0;
    int red_min = 128;
     
    cv::Vec3b & intensity = cv_ptr->image.at<cv::Vec3b>(*j,*i);  // BGR
    if(intensity.val[0] > blue_min &&
       intensity.val[0] < blue_max &&
       intensity.val[1] > green_min &&
       intensity.val[1] < green_max &&
       intensity.val[2] > red_min &&
       intensity.val[2] < red_max){
        return threshold;
    }else{
        return 0;
    }
}

void new_blob(std::list<blob*> *bloblist, int* j, int* i, int* blob_color){
// helper function that adds blob to bloblist
    blob* new_blob = new blob (*j, *i, blob_color);
    (*bloblist).push_back(new_blob);
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
            int threshold = point_check(cv_ptr, &j, &i);  // threshold val  = add to blob, 0 = pass
            int green_color[3] = {0, 255, 0};
            int * green = green_color;
            // check if pixel satisfies condition
            if(threshold != 0){
                if(blobs.size() == 0){   // assign pixels to blob
                    
                    new_blob(&blobs, &j, &i, green);
                }else{
                    std::vector<int> blob_dist;   // iterate through blobs and compute distance to centroids
                    int dist_cnt = 0;
                    for(blobsit k = blobs.begin(); k !=  blobs.end(); ++k){
                        blob_dist.push_back((*k)->check_blob(&j, &i));
                        dist_cnt += 1;
                    }

                    // find minimum distance and compare to threshold
                    int dist_mindex = std::min_element(blob_dist.begin(), blob_dist.end())-blob_dist.begin();
                    int dist_min = *std::min_element(blob_dist.begin(), blob_dist.end());
                    if(dist_min > threshold){
                        new_blob(&blobs, &j, &i, green);
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

