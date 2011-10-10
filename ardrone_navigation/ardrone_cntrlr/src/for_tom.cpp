#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>

using namespace std;

void subcallback(const std_msgs::StringConstPtr& msg)
{
  cout << msg->data.c_str() << "\n";
}


int main(in argc, char **argv)
{

  ros::init(argc, argv, "subscriber_tom");
  ros::NodeHandle n;
  ros::Subscriber sub_handle1 = n.subscribe("topicofchoice", 0, &subcallback);
  ros::Rate loop_rate(60);

  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 1;
}
