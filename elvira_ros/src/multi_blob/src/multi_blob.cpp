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

float * HSV_convert(cv::Vec3b * intensity){
    // https://cs.stackexchange.com/questions/64549/convert-hsv-to-rgb-colors
    float Rp = (float)(*intensity).val[2]/255;
    float Gp = (float)(*intensity).val[1]/255;
    float Bp = (float)(*intensity).val[0]/255;
    float BGRp [] = {Bp, Gp, Rp};
     
    float Cmax = *std::max_element(BGRp, BGRp+3);
    float Cmin = *std::min_element(BGRp, BGRp+3);
    float delta = Cmax - Cmin;

    float H;
    float S;
    float V;

    // compute hue
    if(delta == 0){H = 0.0;
    }else if(Cmax == Rp){H = 60*((Gp-Bp)/delta);
    }else if(Cmax == Gp){H = 60*(2+(Bp-Rp)/delta);
    }else if(Cmax == Bp){H = 60*(4+(Rp-Gp)/delta);
    }

    // compute saturation
    if(Cmax == 0){S=0;
    }else{S = delta/Cmax;
    }

    // compute value
    V = Cmax;

    float * hsv[] = {&H, &S, &V};  
    return * hsv;
}

int point_check(cv_bridge::CvImagePtr& cv_ptr, int*j, int*i){
// helper function that consolidates point to blob assignment conditional check
    int tolerance = 20;  // color tolerance from defined color

    int yellow = 20;
    int orange = 30;
    int purple = 280;  //260 - 300
    int red = 359;

    cv::Vec3b & intensity = cv_ptr->image.at<cv::Vec3b>(*j,*i);  // BGR    

    float * hsv = HSV_convert(&intensity);


    int H = (int)*(hsv);  //hue [0, 360]
    float S = *(hsv+1);   //saturation [0, 1]
    float V = *(hsv+2);   //value [0, 1]
    
    // plot output colors and converted colors
    //ROS_DEBUG_STREAM("R "<< (int)intensity.val[2] << " G " << (int)intensity.val[1] << " B " << (int)intensity.val[0]);
    //ROS_DEBUG_STREAM("H " << H << " S " << S << " V " << V);
    
    // check if certain color in HSV colorspace
    // S = 1 = > pure color (S = 0 => white)
    // V = 0 => gray, V = 1 => white

    //if(S > 0.9 && V < 0.5){    
        if(abs(H-orange) < tolerance && S > 0.8){ return 1;}  // return 1 if orange
        else if(abs(H-yellow) < tolerance && S > 0.4){ return 2;}  // return 2 if yellow
        else if(abs(H-purple) < tolerance){ return 3;}  // return 3 if purple
        else if(abs(H-red) < tolerance && S > 0.8){ return 4;}  // return 4 if red
        else{ return 0;}  // else return 0
}

void new_blob(std::list<blob*> *bloblist, int* j, int* i, int* blob_color){
// helper function that adds blob to bloblist
    blob* new_blob = new blob (*j, *i, blob_color);
    (*bloblist).push_back(new_blob);
}


void pixel_assignment(std::list<blob*> *blobs, int* j, int* i, int* color, int* threshold){
    typedef std::list<blob*>::iterator blobsit;
    if((*blobs).size() == 0){   // assign pixels to blob
        new_blob(blobs, j, i, color);
    }else{
        std::vector<int> blob_dist;   // iterate through blobs and compute distance to centroids
        int dist_cnt = 0;
        for(blobsit k = (*blobs).begin(); k !=  (*blobs).end(); ++k){
            blob_dist.push_back((*k)->check_blob(j, i));
            dist_cnt += 1;
        }
        // find minimum distance and compare to threshold
        int dist_mindex = std::min_element(blob_dist.begin(), blob_dist.end())-blob_dist.begin();
        int dist_min = *std::min_element(blob_dist.begin(), blob_dist.end());
        if(dist_min > *threshold){
            new_blob(blobs, j, i, color);
        }else{
            blobsit it = (*blobs).begin();
            advance(it, dist_mindex);
            (*it)->update(j, i);
        }
    }
}

void drawBlobCircles(std::list<blob*> blobs, cv_bridge::CvImagePtr& cv_ptr){
    typedef std::list<blob*>::iterator blobsit;
    for(blobsit k = blobs.begin(); k != blobs.end(); ++k){
        (*k)->draw_circle(cv_ptr);
    }
}

void multi_blob_track(cv_bridge::CvImagePtr&  cv_ptr){

// enable debugging messages
if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) { // Change the level to fit your needs
   ros::console::notifyLoggerLevelsChanged();
}
    // initialize empty list to store blob objects
    std::list<blob*> orange_blobs = {};
    std::list<blob*> yellow_blobs= {};
    std::list<blob*> purple_blobs = {};
    std::list<blob*> red_blobs = {};
    
    int threshold = 200;  // pixel distance threshold

    // loop through pixels
    for(int j=0; j<cv_ptr->image.rows; j++){
        for(int i=0; i<cv_ptr->image.cols; i++){
            
            int color_code= point_check(cv_ptr, &j, &i);  // threshold val  = add to blob, 0 = pass
            //int orangeBGR[3] = {0, 100, 255};          
            int orangeBGR[3] = {0, 0, 255};          
            int yellowBGR[3] = {0, 255, 255};          
            int purpleBGR[3] = {128, 0, 128};          
            int redBGR[3] = {0, 0, 255};          

            int * orange = orangeBGR;
            int * yellow = yellowBGR;
            int * purple = purpleBGR;
            int * red = redBGR;
 
            // check if pixel satisfies condition
            if(color_code != 0){
                if(color_code == 1){
                    pixel_assignment(&orange_blobs, &j, &i, orangeBGR, &threshold);
                //}else if(color_code == 2){
                //    pixel_assignment(&yellow_blobs, &j, &i, yellowBGR, &threshold);
                }else if(color_code == 3){
                    pixel_assignment(&purple_blobs, &j, &i, purpleBGR, &threshold);
                //}else if(color_code == 4){
                //    pixel_assignment(&red_blobs, &j, &i, redBGR, &threshold);
                }
            }
        }
    } 
    //ROS_DEBUG_STREAM("Frame Iteration Complete");
    //ROS_DEBUG_STREAM(blobs.size());
    
    drawBlobCircles(orange_blobs, cv_ptr);
    drawBlobCircles(yellow_blobs, cv_ptr);
    drawBlobCircles(purple_blobs, cv_ptr);
    drawBlobCircles(red_blobs, cv_ptr);
}

