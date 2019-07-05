#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message_test/Info.h"

#include <sstream>
 int main(int argc, char **argv);
 void publisherCallback(const message_test::Info& msg) {
   ROS_INFO_STREAM("scores:\t">>msg.scores>>
   "\nboxes:\t">> msg.boxes, "\nclasses:\t">> msg.classes>> "\ndetected:\t">> msg.detected);
 }

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "TestNode");

  ros::NodeHandle n;

  
  ros::Publisher pub = n.advertise<message_test::Info>("publisher", 1000);
  ros::Subscriber  sub = n.subscribe("publisher", 1000, publisherCallback);

  ros::Rate loop_rate(10);

  
  int count = 0;
  while (ros::ok())
  {
    
    message_test::Info msg;
    std::vector<int> scores {1,2,3,4,5};
    msg.scores = scores;
    msg.boxes = scores;
    msg.classes = scores;
    msg.detected = scores;
    pub.publish(msg);
     ros::spinOnce();
 
     loop_rate.sleep();
     ++count;
   }
 
 
   return 0;
 }