// this is ripped from tutorial at: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "motion_wrapper.h"

void motorDriverCallback(const std_msgs::String::ConstPtr& msg)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) { // Change the level to fit your needs
   ros::console::notifyLoggerLevelsChanged();
}
    //if(msg->data.compare( "pushup")){
    if(msg->data == "pushup"){
    ROS_DEBUG_STREAM(msg->data.c_str()); 
        sayHello(1);
    }else if(msg->data == "CCW"){
        rotateCCW();
    }else if(msg->data == "CW"){
        rotateCW();
    }else{
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/julia_brain/motor_control", 10, motorDriverCallback);

  ros::spin();
   
    //stanceReset();
    //delay(1000);
    //rotateCWtest();
    //rotateCW();
    //delay(1000);
    //rotateCCW();

  return 0;
}



