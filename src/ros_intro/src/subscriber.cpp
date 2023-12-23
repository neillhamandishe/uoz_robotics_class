#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sstream"


void chatterCallback(const std_msgs::String::ConstPtr &msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main (int argc, char ** argv){
  //create ros node
  ros::init(argc, argv, "cpp_listener");

  ros::NodeHandle n;
  
  //create ros subscriber 
  ros::Subscriber s = n.subscribe("python_chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}

